#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Answers questions about the room: "is there a cooler?", "how far is it?",
"how many people are here?", "what do you see?", "what time is it?"

WHERE THE ANSWERS COME FROM
---------------------------
1. Fast rules first, no model call: go-to commands, time/date, and the
   map questions (count / where / how far / is there). Those are computed
   from the landmark database and live TF -- counts are counted, distances
   measured -- because a map lookup is both instant and exact.
2. The camera + VLM (Cosmos-Reason2 via llama-server on 127.0.0.1:8090,
   started by navigate.launch.py) for anything visual: "what do you see",
   "can you see a bottle", "what colour is the sofa", people in view when
   the detector is not running, and objects that are not on the map.
3. The LLM to classify phrasings the rules cannot, and -- per the operator's
   decision (2026-10-05) -- to answer anything else directly, with the
   current front-camera frame attached when there is one.

Be aware what (3) trades: the model can state things that are not true
(benchmarked: it confidently described a QCar as "an electric vehicle for
urban environments"). Map facts therefore always win when the map has the
answer; the model answers only what the map cannot.

The node still works with no model server: the rule parser handles the
map questions and time on its own.

MAPPED vs LIVE
--------------
Furniture is answered from the saved/being-built map. People are different --
they walk away -- so they are tracked transiently and answered as "right
now", from sightings in the last ~20 s. If you ask about people and the car
has not looked around recently, it says so rather than quietly reporting a
stale number, and it will go and look if you ask it to ("...go and check").

Subscribed:
  /qcar2/objects        std_msgs/String  landmark list + transient + stats
  /qcar2/ask            std_msgs/String  a question, from voice or the console
  /front/camera/preview sensor_msgs/Image latest front frame, for the VLM
Published:
  /qcar2/answer         std_msgs/String  JSON {question, answer, data}
  /qcar2/say            std_msgs/String  spoken answer
  /qcar2/nav_to_object  std_msgs/String  when the question was really a command
  /qcar2/explore_enabled std_msgs/Bool   only for an explicit "go and check"
"""

import base64
import datetime
import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Intents the assistant can answer. When the LLM classifies a question it may
# only pick one of these -- anything else it returns is discarded.
#   time     clock/date, answered locally
#   describe "what do you see" -- camera frame to the VLM
#   look     a visual question about the current view ("can you see a bottle")
#   patrol   "go around the room and ..." (Phase 2; answered from here for now)
#   chat     anything else -- the model answers directly
INTENTS = ('exists', 'count', 'distance', 'where', 'list', 'navigate',
           'time', 'describe', 'look', 'patrol', 'chat', 'unknown')

# Spoken-answer style for every model reply. Kept short because it is read
# aloud by the announcer and shown in the console.
#
# Cosmos-Reason2 is trained on driving footage: told it is "a robot car
# driving around a room" it narrated its own driving ("I am maintaining a
# safe distance") and invented lane markings. So the prompt describes the
# camera as a viewpoint only, and forbids talking about motion.
VLM_SYSTEM = (
    'You are the voice of a small robot (a Quanser QCar 2) in an indoor room. '
    'When an image is attached it is what your front camera sees right now; '
    'speak about it as "I see ...", never "the image". Describe only what is '
    'actually visible. Never talk about your own driving, speed or motion. '
    'Answer in at most two short spoken sentences: no lists, no markdown, '
    'no emojis. If you are not sure, say so. Always reply in English.')

# For questions with no picture. The facts line exists because, asked "what
# is a QCar", the model invented "an electric vehicle for urban environments".
CHAT_SYSTEM = (
    'You are the voice of a small robot car. Facts about yourself: you are a '
    'Quanser QCar 2, a 1/10-scale autonomous research car with a 360 degree '
    'LiDAR, four cameras and an NVIDIA Jetson Orin computer; you map rooms, '
    'recognise objects and drive yourself to them. Answer in at most two short '
    'spoken sentences: no lists, no markdown, no emojis. If you do not know, '
    'say so. Always reply in English.')

# SAFETY SHIELD voice (2026-10-09).  qcar2_safety_shield says a short fixed
# line the instant something happens ("Obstacle in front. Moving back.") --
# safety speech must never wait for a model.  About a second later this adds
# ONE friendly line from the model, built from the shield's real facts, and
# with the camera frame when the car is boxed in or being pestered.  The
# model only talks: it never decides how the car moves (it is ~1 s; the
# shield decides every 50 ms).  Operator choices: playful tone, camera only
# when trapped/pestered, answer "why did you move?" from the facts.
SHIELD_SYSTEM = (
    'You are the voice of a small, friendly robot car (a Quanser QCar 2) at a '
    'live demo. People are testing your safety system by walking, running or '
    'jumping at you; it keeps you from touching anyone by braking and backing '
    'away. You will be given FACTS about what just happened. Speak to the '
    'people around you in one or two short, warm, slightly playful spoken '
    'sentences. Use only the facts given and what is clearly visible; never '
    'invent obstacles, speeds or distances. No lists, no markdown, no emojis. '
    'Always reply in English.')
# A follow-up that arrives after the moment has passed is noise.
SHIELD_FOLLOWUP_STALE_SEC = 5.0
SHIELD_ESCAPE_COOLDOWN_SEC = 10.0     # text-only line after an escape
SHIELD_PESTER_WINDOW_SEC = 30.0
SHIELD_PESTER_COUNT = 3               # escapes in the window = being pestered
SHIELD_PESTER_COOLDOWN_SEC = 30.0
SHIELD_TRAPPED_AFTER_SEC = 1.0        # boxed in this long before the camera line
SHIELD_TRAPPED_REPEAT_SEC = 15.0
SHIELD_CONTEXT_SEC = 60.0             # questions this soon after are about it
SHIELD_Q = re.compile(
    r'\b(why|stuck|trapped|blocked|scared|afraid|run away|running away|ran away|'
    r'moved?|moving|back(ed|ing)? (up|off|away)|escape|escaped|dodge|dodged|'
    r'hit|bump(ed)?|touch(ed)?|ok|okay|alright|hurt|safe|shield)\b')

# Any CJK / Hangul / Cyrillic / Arabic etc. = not English (see chat_completion).
NON_ENGLISH = re.compile(r'[Ѐ-ӿ؀-ۿऀ-ॿ぀-ヿ'
                         r'㐀-鿿가-힯＀-￯]')

# Longest reply read aloud. The model ignores "one sentence" often enough
# (long run-ons joined with semicolons) that the cap is enforced in code.
MAX_SPOKEN_WORDS = 35


def spoken(text):
    """Trim a model reply to something reasonable to say out loud."""
    # Emojis despite being told not to; the speech engine would spell them out.
    text = re.sub(r'[\U0001F000-\U0001FFFF☀-➿️]', '', text)
    text = re.sub(r'\s+', ' ', text).strip()
    words = text.split(' ')
    if len(words) <= MAX_SPOKEN_WORDS:
        return text
    cut = ' '.join(words[:MAX_SPOKEN_WORDS])
    # Prefer ending on a sentence, then a clause, rather than mid-phrase.
    for sep in ('. ', '; ', ', '):
        i = cut.rfind(sep)
        if i > len(cut) // 3:
            return cut[:i].rstrip(',;') + '.'
    return cut + '.'

TIME_Q = re.compile(r'\b(what time|the time|time is it|what day|what date|'
                    r'which day|today s date|the date|what is today)\b')
PATROL_Q = re.compile(r'\b((go|drive|move|look) (a)?round|patrol|search the room|'
                      r'check the (whole )?room|explore the room)\b')
DESCRIBE_Q = re.compile(r'\b(what (do|can) you see|describe|what is in front|'
                        r'what s in front|what are you looking at|look ahead)\b')
LOOK_Q = re.compile(r'\b(do|can) you see\b|\bin front of you\b|\bwhat colou?r\b')

WORD_NUMBERS = {0: 'no', 1: 'one', 2: 'two', 3: 'three', 4: 'four', 5: 'five',
                6: 'six', 7: 'seven', 8: 'eight', 9: 'nine', 10: 'ten'}

# Things people say that mean "people", which the vocabulary calls "person".
PEOPLE_WORDS = ('person', 'people', 'persons', 'human', 'humans', 'anyone',
                'somebody', 'someone')

GO_CHECK = re.compile(
    r'\b(go (and )?(check|look|see)|have a look|check (now|again)|find out)\b', re.I)


NUMBER_WORDS = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6,
                'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
                'first': 1, 'second': 2, 'third': 3, 'fourth': 4, 'fifth': 5,
                'sixth': 6, 'seventh': 7, 'eighth': 8, 'ninth': 9, 'tenth': 10}
_NUM = r'(\d+|' + '|'.join(NUMBER_WORDS) + r')'


def split_number(q):
    """'sofa 2' / 'sofa two' / 'second sofa' -> ('sofa', 2); else (q, None).

    Duplicates are named sofa, sofa 2, sofa 3 (qcar2_object_mapper.py's
    number_names); same parser as qcar2_object_nav.py.
    """
    m = re.match(rf'^(.+?)\s+(?:number\s+)?{_NUM}$', q)
    if m:
        base, num = m.group(1), m.group(2)
    else:
        m = re.match(rf'^{_NUM}\s+(.+)$', q)
        if not m:
            return q, None
        num, base = m.group(1), m.group(2)
    return base, int(num) if num.isdigit() else NUMBER_WORDS[num]


def normalize(text):
    text = (text or '').strip().lower()
    text = re.sub(r'[^a-z0-9\s]', ' ', text)
    return re.sub(r'\s+', ' ', text).strip()


def spoken_count(n):
    return WORD_NUMBERS.get(n, str(n))


def bearing_word(rel_deg):
    """Human direction relative to where the car is pointing."""
    a = abs(rel_deg)
    side = 'left' if rel_deg > 0 else 'right'
    if a <= 25:
        return 'straight ahead'
    if a <= 70:
        return f'ahead and to your {side}'
    if a <= 110:
        return f'to your {side}'
    if a <= 155:
        return f'behind you to the {side}'
    return 'directly behind you'


class Assistant(Node):

    def __init__(self):
        super().__init__('qcar2_assistant')
        share = get_package_share_directory('qcar2_rviz_gui')

        self.declare_parameter('vocabulary_file',
                               os.path.join(share, 'config', 'object_vocabulary.yaml'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_llm', True)
        # llama-server (navigate.launch.py), OpenAI-compatible API.
        self.declare_parameter('llm_url', 'http://127.0.0.1:8090')
        self.declare_parameter('llm_timeout_sec', 20.0)
        # How long to keep waiting for llama-server to finish loading at
        # startup (it starts alongside this node; ~4 s measured for 2B).
        self.declare_parameter('llm_startup_wait_sec', 180.0)
        self.declare_parameter('camera_topic', '/front/camera/preview')
        # A frame older than this is not "what I see now".
        self.declare_parameter('camera_fresh_sec', 2.0)
        self.declare_parameter('live_check_sec', 45.0)
        self.declare_parameter('stale_after_sec', 25.0)

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.use_llm = bool(g('use_llm'))
        self.llm_url = str(g('llm_url')).rstrip('/')
        self.llm_timeout = float(g('llm_timeout_sec'))
        self.llm_startup_wait = float(g('llm_startup_wait_sec'))
        self.camera_fresh = float(g('camera_fresh_sec'))
        self.live_check_sec = float(g('live_check_sec'))
        self.stale_after = float(g('stale_after_sec'))

        self.synonyms, self.vocab = self.load_vocabulary(g('vocabulary_file'))
        self.objects = []
        self.transient = []
        self.stats = {}
        self.last_objects_at = 0.0
        self.llm_ok = None                  # None = not probed yet
        self.lock = threading.Lock()
        # One question at a time: a model call takes up to ~1 s, and two
        # overlapping answers would talk over each other on the speaker.
        self.ask_lock = threading.Lock()
        self.questions_pending = 0
        self.frame = None                   # latest camera Image
        self.frame_at = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.answer_pub = self.create_publisher(String, '/qcar2/answer', 10)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        # Answers to questions are urgent: spoken next, ahead of anything
        # waiting (qcar2_announcer.py), never over a sentence in progress.
        self.say_now_pub = self.create_publisher(String, '/qcar2/say_now', 10)
        self.nav_pub = self.create_publisher(String, '/qcar2/nav_to_object', 10)
        self.explore_pub = self.create_publisher(Bool, '/qcar2/explore_enabled', LATCHED)

        self.create_subscription(String, '/qcar2/objects', self.on_objects, LATCHED)
        self.create_subscription(String, '/qcar2/ask', self.on_ask, 10)
        # Safety shield events -> model follow-ups and question context.
        self.shield_state = 'idle'
        self.shield_info = {}
        self.shield_trapped_since = None
        self.shield_escapes = []            # start times of escape episodes
        self.shield_log = []                # (time, text) of what it did
        self.shield_last_followup = {'escape': 0.0, 'pestered': 0.0, 'trapped': 0.0}
        self.create_subscription(String, '/qcar2/shield_status', self.on_shield_status, LATCHED)
        self.create_subscription(
            Image, g('camera_topic'), self.on_frame,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        if self.use_llm:
            threading.Thread(target=self.probe_llm, daemon=True).start()
        self.get_logger().info('Assistant ready. Ask on /qcar2/ask or from the console.')

    # ------------------------------------------------------------ knowledge

    def load_vocabulary(self, path):
        synonyms, classes = {}, []
        try:
            with open(path) as fh:
                data = yaml.safe_load(fh) or {}
        except OSError as exc:
            self.get_logger().warn(f'Cannot read vocabulary {path}: {exc}')
            return synonyms, classes
        classes = [str(c) for c in (data.get('classes') or [])]
        for canonical, aliases in (data.get('synonyms') or {}).items():
            synonyms[canonical.lower()] = canonical
            for alias in aliases or []:
                synonyms[str(alias).lower()] = canonical
        for c in classes:
            synonyms.setdefault(c.lower(), c)
        return synonyms, classes

    def on_objects(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.objects = data.get('objects') or []
            self.transient = data.get('transient') or []
            self.stats = data.get('stats') or {}
            self.last_objects_at = time.monotonic()

    def on_frame(self, msg):
        # Store only; JPEG encoding happens when a question actually needs it.
        self.frame = msg
        self.frame_at = time.monotonic()

    def camera_jpeg(self):
        """Current front frame as base64 JPEG, or None if there is no fresh one."""
        msg = self.frame
        if msg is None or time.monotonic() - self.frame_at > self.camera_fresh:
            return None
        if msg.encoding not in ('bgr8', 'rgb8'):
            return None
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        # 640 px wide is plenty for scene questions and keeps the vision
        # encoder's token count (and so latency) down.
        if img.shape[1] > 640:
            img = cv2.resize(img, (640, int(img.shape[0] * 640 / img.shape[1])),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(buf.tobytes()).decode() if ok else None

    def car_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:                     # noqa: BLE001 - not localized yet
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def resolve(self, phrase):
        """A spoken noun -> a canonical label that exists on the map."""
        q = normalize(phrase)
        q = re.sub(r'^(any|a|an|the|some)\s+', '', q).strip()
        if not q:
            return None
        if any(re.search(rf'\b{w}\b', q) for w in PEOPLE_WORDS):
            return 'person'
        labels = {o['label'] for o in self.objects} | set(self.vocab)
        canonical = self.synonyms.get(q)
        if canonical:
            return canonical
        # Singularise a plural like "chairs" before giving up.
        for cand in (q, re.sub(r's$', '', q)):
            if cand in self.synonyms:
                return self.synonyms[cand]
            for label in labels:
                if cand == label.lower():
                    return label
        hits = [l for l in labels if q in l.lower() or l.lower() in q]
        if hits:
            return min(hits, key=len)
        return None

    def matching(self, label):
        with self.lock:
            if label == 'person':
                return list(self.transient)
            return [o for o in self.objects if o['label'] == label]

    # -------------------------------------------------------------- parsing

    def parse(self, question):
        """Rule parser first; the LLM only for what the rules cannot handle."""
        q = normalize(question)
        if not q:
            return {'intent': 'unknown'}

        if TIME_Q.search(q):
            return {'intent': 'time'}
        # Before navigate: "go around the room" is a task, not a destination.
        if PATROL_Q.search(q):
            m = re.search(r'\bhow many\s+(.+?)(?:\s+are|\s+is|\s+there|$)', q)
            return {'intent': 'patrol', 'target': m.group(1) if m else ''}
        if re.search(r'\b(go|drive|take me|navigate)\b.*\bto\b', q):
            m = re.search(r'\bto\s+(?:the\s+|a\s+|an\s+)?(.+)$', q)
            return {'intent': 'navigate', 'target': m.group(1) if m else ''}
        # Before list: "what do you see" means the CAMERA now, not the map.
        if DESCRIBE_Q.search(q):
            return {'intent': 'describe'}
        if re.search(r'\b(what|which)\b.*\b(mapped|objects|in the room|there|know)\b', q) \
                or re.search(r'\blist\b', q):
            return {'intent': 'list'}
        # "Can you see a bottle?" / "what colour is the sofa?" -> camera.
        if LOOK_Q.search(q):
            return {'intent': 'look'}
        m = re.search(r'\bhow many\s+(.+?)(?:\s+are|\s+is|\s+do|\s+can|\?|$)', q)
        if m:
            return {'intent': 'count', 'target': m.group(1)}
        m = re.search(r'\b(how far|what.*distance|how close)\b.*?'
                      r'\b(?:to|from|is)\s+(?:the\s+|a\s+|an\s+)?(.+)$', q)
        if m:
            return {'intent': 'distance', 'target': m.group(2)}
        m = re.search(r'\bwhere\s+(?:is|are)\s+(?:the\s+|a\s+|an\s+)?(.+)$', q)
        if m:
            return {'intent': 'where', 'target': m.group(1)}
        # Anchored to the START: unanchored, the "is" in "what is a qcar"
        # turned a general question into "is there a qcar in the room".
        m = re.search(r'^(?:is|are|have you seen)\s+'
                      r'(?:there\s+)?(?:any\s+|a\s+|an\s+|the\s+)?(.+?)'
                      r'(?:\s+in\s+the\s+room|\s+here|\s+around|\?|$)', q)
        if m:
            return {'intent': 'exists', 'target': m.group(1)}
        if self.use_llm and self.llm_ok:
            parsed = self.ask_llm(question)
            if parsed:
                return parsed
            return {'intent': 'chat'}
        return {'intent': 'unknown'}

    # ------------------------------------------------------------------ llm

    def probe_llm(self):
        """Wait for llama-server to finish loading, then warm it up.

        It is started by the same launch as this node, so "not up yet" is the
        normal state for the first few seconds, not a failure. Questions asked
        meanwhile fall back to the rule parser instead of blocking.
        """
        deadline = time.monotonic() + self.llm_startup_wait
        while time.monotonic() < deadline and rclpy.ok():
            try:
                with urllib.request.urlopen(self.llm_url + '/health', timeout=2.0) as resp:
                    if json.loads(resp.read().decode()).get('status') == 'ok':
                        break
            except Exception:                 # noqa: BLE001 - still loading / not started
                pass
            time.sleep(1.0)
        else:
            self.llm_ok = False
            self.get_logger().info(
                f'No model server at {self.llm_url}; using the rule parser only '
                f'(navigate.launch.py use_llm:=true starts it).')
            return
        # The very first image request pays a one-off ~3 s GPU warm-up
        # (benchmarked: 3.9 s first vs 0.85 s after). Spend it now, on a
        # blank frame, rather than on the operator's first question.
        blank = cv2.imencode('.jpg', np.zeros((64, 64, 3), np.uint8))[1].tobytes()
        self.chat_completion('Say OK.', base64.b64encode(blank).decode(), max_tokens=4)
        self.llm_ok = True
        self.get_logger().info('Vision-language model ready for questions.')

    def chat_completion(self, prompt, image_b64=None, max_tokens=120, system=None,
                        temperature=0.2, _retry=False):
        """One request to llama-server; returns the reply text or None."""
        content = [{'type': 'text', 'text': prompt}]
        if image_b64:
            content.insert(0, {'type': 'image_url',
                               'image_url': {'url': f'data:image/jpeg;base64,{image_b64}'}})
        messages = [{'role': 'user', 'content': content}]
        if system:
            messages.insert(0, {'role': 'system', 'content': system})
        body = json.dumps({'messages': messages, 'max_tokens': max_tokens,
                           'temperature': temperature}).encode()
        try:
            req = urllib.request.Request(self.llm_url + '/v1/chat/completions', data=body,
                                         headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=self.llm_timeout) as resp:
                out = json.loads(resp.read().decode())
        except Exception as exc:              # noqa: BLE001 - degrade, never die
            self.get_logger().warn(f'Model request failed: {exc}')
            return None
        text = (out['choices'][0]['message'].get('content') or '').strip()
        # Cosmos-Reason is a reasoning model; never read its scratchpad aloud.
        text = re.sub(r'<think>.*?</think>', '', text, flags=re.S).strip()
        # ENGLISH ONLY.  Cosmos-Reason2 is built on a Chinese/English model;
        # a playful prompt at temperature 0.7 came back entirely in Chinese
        # (2026-10-09), which the English voice could not even play.  One
        # retry with the instruction repeated, then give up rather than speak.
        if NON_ENGLISH.search(text):
            if not _retry:
                self.get_logger().warn(f'Model replied in another language, retrying: {text[:60]}')
                return self.chat_completion(
                    prompt + '\n\nReply in English only.', image_b64, max_tokens, system,
                    temperature=min(temperature, 0.2), _retry=True)
            self.get_logger().warn('Model replied in another language twice; not spoken.')
            return None
        return text or None

    def ask_llm(self, question):
        """Free text -> a structured intent, validated before use."""
        labels = sorted({o['label'] for o in self.objects}) or self.vocab[:20]
        prompt = (
            'Classify this request to a robot car into JSON. Reply with JSON only.\n'
            'Schema: {"intent": one of ["exists","count","distance","where","list",'
            '"navigate","time","describe","look","patrol","chat"], '
            '"target": "<object name or empty>"}\n'
            'navigate = drive to an object; patrol = drive around the room to do '
            'something; describe = what the camera sees; look = a question about '
            'the current camera view; chat = anything else.\n'
            f'Objects on the map: {", ".join(labels)}\n'
            f'Request: {question}')
        out = self.chat_completion(prompt, max_tokens=60, temperature=0.0)
        if not out:
            return None
        m = re.search(r'\{.*\}', out, re.S)
        if not m:
            return None
        try:
            parsed = json.loads(m.group(0))
        except ValueError:
            return None
        # Validate: the model may only choose among intents we implement.
        intent = str(parsed.get('intent', '')).lower()
        if intent not in INTENTS or intent == 'unknown':
            return None
        return {'intent': intent, 'target': str(parsed.get('target', ''))}

    # ------------------------------------------------------------- answering

    def on_ask(self, msg):
        question = (msg.data or '').strip()
        if not question:
            return
        # Off the executor thread: a model call can take ~1 s, and blocking
        # here would also stop camera frames arriving in on_frame().
        with self.lock:
            self.questions_pending += 1     # a shield follow-up yields to this
        threading.Thread(target=self.handle_question, args=(question,), daemon=True).start()

    def handle_question(self, question):
        try:
            with self.ask_lock:
                self._handle_question(question)
        finally:
            with self.lock:
                self.questions_pending -= 1

    def _handle_question(self, question):
        try:
            answer, data = self.answer(question)
        except Exception as exc:              # noqa: BLE001 - never die on a question
            self.get_logger().error(f'Failed to answer "{question}": {exc}')
            answer, data = "Sorry, I could not work that out.", {'error': str(exc)}
        self.get_logger().info(f'Q: "{question}" -> {answer}')
        self.say(answer, urgent=True)
        out = String()
        out.data = json.dumps({'question': question, 'answer': answer, 'data': data})
        self.answer_pub.publish(out)

    def answer(self, question):
        # Asked about it right after the shield acted ("why did you move?",
        # "are you stuck?"): answer from what it really saw.
        if (self.llm_ok and self.shield_log and
                time.monotonic() - self.shield_log[-1][0] < SHIELD_CONTEXT_SEC and
                SHIELD_Q.search(normalize(question))):
            text = self.chat_completion(
                f'FACTS (most recent last):\n{self.shield_facts()}\n\n'
                f'Someone just asked you: "{question}". Answer them.',
                system=SHIELD_SYSTEM, max_tokens=80, temperature=0.4)
            if text:
                return spoken(text), {'intent': 'shield', 'source': 'llm'}
        parsed = self.parse(question)
        intent = parsed.get('intent', 'unknown')
        target = parsed.get('target', '')

        if intent == 'time':
            return self.answer_time()
        if intent == 'list':
            return self.answer_list()
        if intent == 'navigate':
            label = self.resolve(target)
            out = String()
            out.data = target
            self.nav_pub.publish(out)
            return (f'Going to the {label or target}.',
                    {'intent': 'navigate', 'target': label or target})
        if intent == 'patrol':
            return self.answer_patrol(question, target)
        if intent == 'describe':
            # One sentence: it is spoken aloud, and "one or two" came back as
            # three long ones.
            return self.answer_visual(
                'In one short sentence, what do you see?', 'describe')
        if intent in ('look', 'chat'):
            return self.answer_visual(question, intent)
        if intent == 'unknown':
            return ("I can tell you the time, what I have mapped, how many there are, "
                    "where something is, or how far away it is.", {'intent': 'unknown'})

        # "how far is sofa 2" -> that sofa, not the nearest one.
        base, number = split_number(normalize(target))
        label = self.resolve(base)
        if label is None:
            # Not a map object -- "how many bottles", "is there a cat". The
            # camera may still answer it for what is in view right now.
            if self.llm_ok:
                text, data = self.answer_visual(question, 'look')
                if data.get('camera'):
                    text = f'That is not on my map, but looking ahead: {text}'
                data['unresolved'] = target
                return text, data
            known = sorted({o['label'] for o in self.objects})
            return (f'I do not know what "{target}" is. I have mapped: '
                    f'{", ".join(known) if known else "nothing yet"}.',
                    {'intent': intent, 'unresolved': target})

        found = self.matching(label)
        if label == 'person':
            return self.answer_people(question, found)
        if number is not None and intent in ('distance', 'where', 'exists'):
            want = label if number == 1 else f'{label} {number}'
            exact = [o for o in found if o.get('name', o['label']) == want]
            if not exact:
                names = ', '.join(o.get('name', label) for o in found) or 'none'
                return (f'There is no {want}. I have: {names}.',
                        {'intent': intent, 'label': label, 'found': False})
            found = exact
        if intent == 'count':
            return self.answer_count(label, found)
        if intent == 'exists':
            return self.answer_exists(label, found)
        if intent in ('distance', 'where'):
            return self.answer_where(label, found, include_direction=(intent == 'where'))
        return ("I can tell you what I have seen, how many there are, where "
                "something is, or how far away it is.", {'intent': 'unknown'})

    def answer_time(self):
        now = datetime.datetime.now()
        hour = now.strftime('%I').lstrip('0')
        d = now.day
        suffix = 'th' if 10 <= d % 100 <= 20 else {1: 'st', 2: 'nd', 3: 'rd'}.get(d % 10, 'th')
        text = f'It is {hour}:{now:%M} {now:%p}, {now:%A} the {d}{suffix} of {now:%B}.'
        return text, {'intent': 'time', 'iso': now.isoformat(timespec='seconds')}

    def answer_visual(self, question, intent):
        """Answer from the front camera (or as plain chat if there is no frame)."""
        if not self.llm_ok:
            return ('My vision model is not running, so I can only answer questions '
                    'about the map right now.', {'intent': intent, 'llm': False})
        # General chat gets no picture: with one attached, "what is a qcar"
        # was answered by describing the people in view.
        image = self.camera_jpeg() if intent != 'chat' else None
        if image is None and intent in ('describe', 'look'):
            return ('I am not getting a picture from my front camera right now.',
                    {'intent': intent, 'camera': False})
        text = self.chat_completion(question, image,
                                    system=VLM_SYSTEM if image else CHAT_SYSTEM)
        if not text:
            return ('Sorry, I could not work that out.', {'intent': intent, 'error': 'model'})
        return spoken(text), {'intent': intent, 'camera': image is not None, 'source': 'vlm'}

    def answer_patrol(self, question, target):
        # Phase 2 adds the drive-around (viewpoints from the map + detector +
        # VLM). Until then, say so and answer from the current view instead of
        # silently doing nothing.
        if not self.llm_ok or self.camera_jpeg() is None:
            return ('I cannot drive around the room for that yet. Ask me what I can '
                    'see, or send me to an object.', {'intent': 'patrol', 'target': target})
        text, data = self.answer_visual(question, 'look')
        data.update(intent='patrol', target=target, patrol=False)
        return ('I cannot drive around the room for that yet, but from where I am: '
                + text, data)

    def answer_list(self):
        with self.lock:
            objects, transient = list(self.objects), list(self.transient)
        if not objects:
            return ('I have not mapped anything yet.', {'intent': 'list', 'objects': []})
        counts = {}
        for o in objects:
            counts[o['label']] = counts.get(o['label'], 0) + 1
        parts = [(f'{spoken_count(n)} {label}' + ('s' if n > 1 and not label.endswith('s') else ''))
                 for label, n in sorted(counts.items())]
        text = 'I have mapped ' + ', '.join(parts[:-1])
        text = (text + ', and ' + parts[-1]) if len(parts) > 1 else 'I have mapped ' + parts[0]
        if transient:
            text += f'. I can also see {spoken_count(len(transient))} ' \
                    f'{"person" if len(transient) == 1 else "people"} right now'
        return text + '.', {'intent': 'list', 'counts': counts,
                            'people_now': len(transient)}

    def answer_count(self, label, found):
        n = len(found)
        noun = label if n == 1 else (label + ('' if label.endswith('s') else 's'))
        if n == 0:
            return (f'I have not seen any {noun} in this room.',
                    {'intent': 'count', 'label': label, 'count': 0})
        return (f'There {"is" if n == 1 else "are"} {spoken_count(n)} {noun}.',
                {'intent': 'count', 'label': label, 'count': n})

    def answer_exists(self, label, found):
        if not found:
            return (f'No, I have not seen a {label} in this room.',
                    {'intent': 'exists', 'label': label, 'exists': False})
        text, data = self.answer_where(label, found, include_direction=True)
        n = len(found)
        prefix = f'Yes. ' if n == 1 else f'Yes, there are {spoken_count(n)}. '
        data['exists'] = True
        return prefix + text, data

    def answer_where(self, label, found, include_direction):
        if not found:
            return (f'I have not seen a {label}.',
                    {'intent': 'where', 'label': label, 'found': False})
        pose = self.car_pose()
        if pose is None:
            return (f'I know where the {label} is on the map, but I do not know '
                    f'where I am yet, so I cannot measure the distance.',
                    {'intent': 'where', 'label': label, 'no_pose': True})
        cx, cy, yaw = pose
        best = min(found, key=lambda o: math.hypot(o['x'] - cx, o['y'] - cy))
        dx, dy = best['x'] - cx, best['y'] - cy
        dist = math.hypot(dx, dy)
        rel = math.degrees(math.atan2(dy, dx) - yaw)
        rel = (rel + 180) % 360 - 180
        # Confidence in the NAME is worth surfacing: the map can be sure
        # something is there while being unsure what it is.
        hedge = ''
        if best.get('label_confidence', 1.0) < 0.6 and best.get('alternatives'):
            hedge = f" -- though I am not certain; it might be a {best['alternatives'][0]['label']}"
        # Say WHICH one ("The sofa 3 is ...") so it matches the map label.
        text = f'The {best.get("name", label)} is {dist:.1f} metres away'
        if include_direction:
            text += f', {bearing_word(rel)}'
        return text + hedge + '.', {
            'intent': 'where', 'label': label, 'found': True,
            'distance_m': round(dist, 2), 'bearing_deg': round(rel, 1),
            'x': best['x'], 'y': best['y'], 'count': len(found),
            'label_confidence': best.get('label_confidence'),
        }

    def answer_people(self, question, found):
        """People are answered as 'now', never from a stale map."""
        n = len(found)
        age = time.monotonic() - self.last_objects_at if self.last_objects_at else 1e9
        wants_live = bool(GO_CHECK.search(question))

        # The explorer only runs in autonomous mapping. In navigation mode
        # nothing subscribes, and publishing would just wait 45 s and report
        # the same stale number -- so only ask when someone is listening.
        if wants_live and self.explore_pub.get_subscription_count() > 0:
            self.request_live_check()
            return ('Let me go and look around, I will tell you what I find.',
                    {'intent': 'count', 'label': 'person', 'live_check': True,
                     'count_now': n})
        if wants_live:
            return self.answer_patrol(question, 'person')
        # No live detector (navigation mode runs none): count what the front
        # camera sees right now with the VLM rather than report a stale list.
        if age > self.stale_after and self.llm_ok and self.camera_jpeg() is not None:
            reply = self.chat_completion(
                'How many people are in this image? Reply with just the number.',
                self.camera_jpeg(), max_tokens=6, temperature=0.0)
            m = re.search(r'\d+', reply or '')
            if m:
                k = int(m.group(0))
                text = (f'Looking ahead, I can see {spoken_count(k)} '
                        f'{"person" if k == 1 else "people"}.' if k else
                        'I cannot see anybody in front of me right now.')
                return (text + ' I only see what my front camera is pointed at.',
                        {'intent': 'count', 'label': 'person', 'count': k,
                         'source': 'vlm', 'camera': True})
        if age > self.stale_after:
            return (f'I am not getting live camera updates right now, so I cannot '
                    f'say for sure. The last time I looked I could see '
                    f'{spoken_count(n)} {"person" if n == 1 else "people"}. '
                    f'Ask me to go and check if you want a fresh count.',
                    {'intent': 'count', 'label': 'person', 'stale': True, 'count': n})
        if n == 0:
            return ('I cannot see anybody right now. Bear in mind I only see what '
                    'my cameras are pointed at -- ask me to go and check to be sure.',
                    {'intent': 'count', 'label': 'person', 'count': 0})
        return (f'I can see {spoken_count(n)} {"person" if n == 1 else "people"} right now.',
                {'intent': 'count', 'label': 'person', 'count': n})

    def request_live_check(self):
        """Ask the explorer to sweep the room, then report the fresh count."""
        msg = Bool()
        msg.data = True
        self.explore_pub.publish(msg)

        def report():
            time.sleep(self.live_check_sec)
            found = self.matching('person')
            n = len(found)
            self.say(f'I have had a look. I can see {spoken_count(n)} '
                     f'{"person" if n == 1 else "people"}.')
            out = String()
            out.data = json.dumps({'question': 'live person check',
                                   'answer': f'{n} people', 'data': {'count': n}})
            self.answer_pub.publish(out)

        threading.Thread(target=report, daemon=True).start()

    def say(self, text, urgent=False):
        msg = String()
        msg.data = text
        (self.say_now_pub if urgent else self.say_pub).publish(msg)

    # -------------------------------------------------------- safety shield

    def on_shield_status(self, msg):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        state = info.get('state', 'idle')
        prev = self.shield_state
        self.shield_state, self.shield_info = state, info
        acting = ('escape', 'hold', 'trapped')
        if state in ('escape', 'trapped') and info.get('text'):
            if not self.shield_log or self.shield_log[-1][1] != info['text']:
                self.shield_log = (self.shield_log + [(now, info['text'])])[-8:]
        elif state == 'idle' and prev in acting and self.shield_log:
            self.shield_log = (self.shield_log + [(now, 'Clear again, carrying on.')])[-8:]

        self.shield_trapped_since = (self.shield_trapped_since or now) if state == 'trapped' else None
        if not self.llm_ok:
            return                      # the shield's fixed lines already cover it
        last = self.shield_last_followup
        if state == 'escape' and prev != 'escape':
            self.shield_escapes = [t for t in self.shield_escapes
                                   if now - t < SHIELD_PESTER_WINDOW_SEC] + [now]
            if (len(self.shield_escapes) >= SHIELD_PESTER_COUNT and
                    now - last['pestered'] > SHIELD_PESTER_COOLDOWN_SEC):
                self.shield_followup('pestered', now)
            elif now - last['escape'] > SHIELD_ESCAPE_COOLDOWN_SEC:
                self.shield_followup('escape', now)
        if (self.shield_trapped_since is not None and
                now - self.shield_trapped_since >= SHIELD_TRAPPED_AFTER_SEC and
                now - last['trapped'] > SHIELD_TRAPPED_REPEAT_SEC):
            self.shield_followup('trapped', now)

    def shield_facts(self):
        now = time.monotonic()
        lines = [f'{now - t:.0f} s ago: {text}' for t, text in self.shield_log]
        info = self.shield_info
        gap = info.get('gap')
        if isinstance(gap, (int, float)) and gap < 1.0:
            lines.append(f'Closest thing right now: {gap * 100:.0f} cm from my body.')
        n = len([t for t in self.shield_escapes if now - t < SHIELD_PESTER_WINDOW_SEC])
        if n >= 2:
            lines.append(f'I have had to dodge {n} times in the last half minute.')
        return '\n'.join(lines) or 'Nothing has happened yet.'

    def shield_followup(self, kind, at):
        self.shield_last_followup[kind] = at
        threading.Thread(target=self._shield_followup, args=(kind, at), daemon=True).start()

    def _shield_followup(self, kind, at):
        # A question being answered (or waiting) comes first: skip rather
        # than queue a line that would arrive long after the moment.
        if self.questions_pending or not self.ask_lock.acquire(blocking=False):
            return
        try:
            image = self.camera_jpeg() if kind in ('trapped', 'pestered') else None
            ask = {
                'escape': 'Add one short friendly line about what you just did.',
                'pestered': 'People keep coming at you. Playfully ask them to give '
                            'you some space.',
                'trapped': 'You are boxed in and cannot move. Kindly ask the people '
                           'around you to step back so you can get out.',
            }[kind]
            if image:
                ask += (' The attached image is your front camera right now; you may '
                        'mention who or what you see in front of you, only if it is '
                        'clearly visible.')
            text = self.chat_completion(f'FACTS (most recent last):\n{self.shield_facts()}\n\n{ask}',
                                        image, system=SHIELD_SYSTEM, max_tokens=70,
                                        temperature=0.4)
            if not text:
                return
            # Still relevant?  Freed already = do not ask them to move; and
            # someone asked something meanwhile = their answer goes first.
            if self.questions_pending:
                return
            now = time.monotonic()
            if kind == 'trapped' and self.shield_state != 'trapped':
                return
            if kind != 'trapped' and now - at > SHIELD_FOLLOWUP_STALE_SEC:
                return
            text = spoken(text)
            self.get_logger().info(f'Shield follow-up ({kind}): {text}')
            self.say(text)
        finally:
            self.ask_lock.release()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Assistant()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
