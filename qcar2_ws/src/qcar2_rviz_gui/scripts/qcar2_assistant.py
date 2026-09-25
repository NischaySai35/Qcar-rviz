#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Answers questions about the room: "is there a cooler?", "how far is it?",
"how many people are here?"

WHERE THE ANSWERS COME FROM -- AND WHY NOT FROM THE LLM
-------------------------------------------------------
Every factual answer is computed from the landmark database and live TF:
counts are counted, distances are measured from the car's current pose. The
optional local LLM is used ONLY to turn free-form wording into a structured
query when the rule parser cannot.

That split is deliberate. A small language model asked "how many chairs are
in here" will happily produce a confident number it invented, and a robot
that makes up facts about its surroundings is worse than one that says "I
don't know". So the LLM may choose the QUESTION, never the ANSWER. Everything
it returns is validated against the known intents and the real vocabulary
before anything is acted on.

The whole node works with no LLM installed: the rule parser handles the
question shapes people actually use, and the LLM only widens what phrasings
are understood. `scripts/install_llm.sh` adds it.

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
Published:
  /qcar2/answer         std_msgs/String  JSON {question, answer, data}
  /qcar2/say            std_msgs/String  spoken answer
  /qcar2/nav_to_object  std_msgs/String  when the question was really a command
  /qcar2/explore_enabled std_msgs/Bool   only for an explicit "go and check"
"""

import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Intents the assistant can answer. The LLM is only ever allowed to pick one
# of these -- anything else it returns is discarded.
INTENTS = ('exists', 'count', 'distance', 'where', 'list', 'navigate', 'unknown')

WORD_NUMBERS = {0: 'no', 1: 'one', 2: 'two', 3: 'three', 4: 'four', 5: 'five',
                6: 'six', 7: 'seven', 8: 'eight', 9: 'nine', 10: 'ten'}

# Things people say that mean "people", which the vocabulary calls "person".
PEOPLE_WORDS = ('person', 'people', 'persons', 'human', 'humans', 'anyone',
                'somebody', 'someone')

GO_CHECK = re.compile(
    r'\b(go (and )?(check|look|see)|have a look|check (now|again)|find out)\b', re.I)


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
        self.declare_parameter('llm_url', 'http://127.0.0.1:11434/api/generate')
        self.declare_parameter('llm_model', 'qwen2.5:1.5b-instruct')
        self.declare_parameter('llm_timeout_sec', 6.0)
        self.declare_parameter('live_check_sec', 45.0)
        self.declare_parameter('stale_after_sec', 25.0)

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.base_frame = g('base_frame')
        self.use_llm = bool(g('use_llm'))
        self.llm_url = g('llm_url')
        self.llm_model = g('llm_model')
        self.llm_timeout = float(g('llm_timeout_sec'))
        self.live_check_sec = float(g('live_check_sec'))
        self.stale_after = float(g('stale_after_sec'))

        self.synonyms, self.vocab = self.load_vocabulary(g('vocabulary_file'))
        self.objects = []
        self.transient = []
        self.stats = {}
        self.last_objects_at = 0.0
        self.llm_ok = None                  # None = not probed yet
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.answer_pub = self.create_publisher(String, '/qcar2/answer', 10)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        self.nav_pub = self.create_publisher(String, '/qcar2/nav_to_object', 10)
        self.explore_pub = self.create_publisher(Bool, '/qcar2/explore_enabled', LATCHED)

        self.create_subscription(String, '/qcar2/objects', self.on_objects, LATCHED)
        self.create_subscription(String, '/qcar2/ask', self.on_ask, 10)

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

        if re.search(r'\b(go|drive|take me|navigate)\b.*\bto\b', q):
            m = re.search(r'\bto\s+(?:the\s+|a\s+|an\s+)?(.+)$', q)
            return {'intent': 'navigate', 'target': m.group(1) if m else ''}
        if re.search(r'\b(what|which)\b.*\b(see|there|room|objects|around|know)\b', q) \
                or re.search(r'\blist\b', q):
            return {'intent': 'list'}
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
        m = re.search(r'\b(?:is|are|do you see|can you see|have you seen)\s+'
                      r'(?:there\s+)?(?:any\s+|a\s+|an\s+|the\s+)?(.+?)'
                      r'(?:\s+in\s+the\s+room|\s+here|\s+around|\?|$)', q)
        if m:
            return {'intent': 'exists', 'target': m.group(1)}
        if self.use_llm and self.llm_ok:
            parsed = self.ask_llm(question)
            if parsed:
                return parsed
        return {'intent': 'unknown'}

    # ------------------------------------------------------------------ llm

    def probe_llm(self):
        """One-shot availability check, so questions never block on a dead LLM."""
        try:
            base = self.llm_url.rsplit('/api/', 1)[0] + '/api/tags'
            with urllib.request.urlopen(base, timeout=3.0) as resp:
                tags = json.loads(resp.read().decode())
            names = [m.get('name', '') for m in tags.get('models', [])]
            self.llm_ok = any(n.startswith(self.llm_model.split(':')[0]) for n in names)
            if self.llm_ok:
                self.get_logger().info(f'LLM available ({self.llm_model}) for free-form questions.')
            else:
                self.get_logger().info(
                    f'LLM server is up but "{self.llm_model}" is not pulled; using the rule '
                    f'parser. Run scripts/install_llm.sh.')
        except Exception:                     # noqa: BLE001 - absence is normal
            self.llm_ok = False
            self.get_logger().info(
                'No local LLM found; using the rule parser (run scripts/install_llm.sh '
                'to widen the phrasings understood).')

    def ask_llm(self, question):
        """Free text -> a structured intent. The LLM never supplies facts."""
        labels = sorted({o['label'] for o in self.objects}) or self.vocab[:20]
        prompt = (
            'Convert the question into JSON describing what is being asked. '
            'Reply with JSON only, no prose.\n'
            'Schema: {"intent": one of ["exists","count","distance","where","list","navigate"], '
            '"target": "<object name or empty>"}\n'
            f'Known objects: {", ".join(labels)}\n'
            f'Question: {question}\nJSON:')
        body = json.dumps({
            'model': self.llm_model, 'prompt': prompt, 'stream': False,
            'options': {'temperature': 0.0, 'num_predict': 80},
        }).encode()
        try:
            req = urllib.request.Request(
                self.llm_url, data=body, headers={'Content-Type': 'application/json'})
            with urllib.request.urlopen(req, timeout=self.llm_timeout) as resp:
                out = json.loads(resp.read().decode()).get('response', '')
        except Exception as exc:              # noqa: BLE001 - degrade to rules
            self.get_logger().warn(f'LLM query failed ({exc}); using the rule parser.')
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
        try:
            answer, data = self.answer(question)
        except Exception as exc:              # noqa: BLE001 - never die on a question
            self.get_logger().error(f'Failed to answer "{question}": {exc}')
            answer, data = "Sorry, I could not work that out.", {'error': str(exc)}
        self.get_logger().info(f'Q: "{question}" -> {answer}')
        self.say(answer)
        out = String()
        out.data = json.dumps({'question': question, 'answer': answer, 'data': data})
        self.answer_pub.publish(out)

    def answer(self, question):
        parsed = self.parse(question)
        intent = parsed.get('intent', 'unknown')
        target = parsed.get('target', '')

        if intent == 'list':
            return self.answer_list()
        if intent == 'navigate':
            label = self.resolve(target)
            out = String()
            out.data = target
            self.nav_pub.publish(out)
            return (f'Going to the {label or target}.',
                    {'intent': 'navigate', 'target': label or target})

        label = self.resolve(target)
        if label is None:
            known = sorted({o['label'] for o in self.objects})
            return (f'I do not know what "{target}" is. I have mapped: '
                    f'{", ".join(known) if known else "nothing yet"}.',
                    {'intent': intent, 'unresolved': target})

        found = self.matching(label)
        if label == 'person':
            return self.answer_people(question, found)
        if intent == 'count':
            return self.answer_count(label, found)
        if intent == 'exists':
            return self.answer_exists(label, found)
        if intent in ('distance', 'where'):
            return self.answer_where(label, found, include_direction=(intent == 'where'))
        return ("I can tell you what I have seen, how many there are, where "
                "something is, or how far away it is.", {'intent': 'unknown'})

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
        text = f'The {label} is {dist:.1f} metres away'
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

        if wants_live:
            self.request_live_check()
            return ('Let me go and look around, I will tell you what I find.',
                    {'intent': 'count', 'label': 'person', 'live_check': True,
                     'count_now': n})
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

    def say(self, text):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)


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
