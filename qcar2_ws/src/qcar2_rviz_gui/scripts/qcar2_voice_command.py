#!/usr/bin/python3
# Hardcoded, not `env python3`: see image_preview_throttle.py for why --
# `env python3` can resolve to pyenv's Python 3.7 instead of the system
# 3.8 rclpy's C extension needs, depending on shell PATH state.
"""Spoken commands: "hey car, go to the air cooler".

Offline speech recognition (Vosk) with two selectable microphones, gated by a
wake phrase so ordinary conversation in the room is ignored.

WHY IT DOES NOT REACT TO NORMAL CONVERSATION
--------------------------------------------
Three independent filters, because any one alone is not enough for something
that is always listening:

1. WAKE PHRASE. Nothing is acted on unless the utterance contains "hey car".
   Saying "I should go to the air cooler" to another person does nothing.
2. RESTRICTED GRAMMAR. Vosk is given an explicit word list (the wake phrase,
   the command verbs, and the mapped object names) instead of open
   vocabulary. It then cannot emit words outside that list, so unrelated
   speech collapses to silence or "[unk]" rather than being force-fitted onto
   the nearest command. This also makes recognition markedly faster and more
   accurate on the words that DO matter, which is why it is worth the
   restriction.
3. INTENT MATCH. What survives must still parse as a known command, and the
   object name must resolve against the map. Anything else is dropped.

The one deliberate exception is a bare "stop", which is honoured without the
wake phrase (configurable). A spurious stop costs nothing -- the car halts --
whereas needing a wake phrase during an emergency is a genuinely bad trade.

TWO MICROPHONES
---------------
  car      -- the QCar2's onboard mic via PortAudio/PulseAudio. Talk to the
              car in the room; needs no browser tab open.
  browser  -- the laptop/phone viewing the web console streams 16 kHz PCM
              over the existing WebSocket; qcar2_web_gui.py republishes it on
              /qcar2/mic_audio. Better range and mic quality.
Both feed the SAME recogniser, so behaviour is identical either way.

Subscribed:
  /qcar2/mic_enabled  std_msgs/Bool             master on/off (default OFF)
  /qcar2/mic_source   std_msgs/String           'car' or 'browser'
  /qcar2/mic_audio    std_msgs/UInt8MultiArray  16 kHz mono s16le from browser
Published:
  /qcar2/nav_to_object  std_msgs/String   object name -> qcar2_object_nav
  /qcar2/voice_transcript std_msgs/String JSON for the web console
  /qcar2/say            std_msgs/String   spoken replies
  /qcar2_estop          std_msgs/Bool     emergency stop
"""

import json
import math
import os
import queue
from collections import deque
import re
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.srv import CancelGoal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String, UInt8MultiArray

LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

SAMPLE_RATE = 16000

# Command verbs. Kept as whole phrases so the grammar biases toward them.
GO_PATTERNS = re.compile(
    r'\b(go|drive|move|navigate|head|take me|get)\b.*?\b(to|towards|toward|near)\b\s*(.*)', re.I)
STOP_WORDS = ('stop', 'halt', 'freeze', 'stop now')
CANCEL_WORDS = ('cancel', 'cancel goal', 'never mind', 'abort')
RESUME_WORDS = ('resume', 'continue', 'carry on', 'release')
LIST_WORDS = ('what can you see', 'list objects', 'what do you see', 'what objects')

# Anything matching these is a QUESTION, not a drive command, and is handed to
# qcar2_assistant.py to answer from the map. Checked before the "go to X"
# pattern, because "how far is the sofa" also contains a bare object name and
# would otherwise be read as an order to drive there.
QUESTION_PATTERNS = re.compile(
    r'\b(how many|how far|how close|how much|where is|where are|is there|are there|'
    r'do you see|can you see|have you seen|what is|what are|what can you|'
    r'what do you|which|anyone|anybody|distance to)\b', re.I)

# Words the grammar always needs regardless of the object vocabulary.
BASE_WORDS = (
    'hey car go drive move navigate head take me get to towards toward near '
    'the a an my stop halt freeze cancel goal never mind abort resume continue '
    'carry on release what can you see do list objects yes no please '
    # Question vocabulary. The recogniser is restricted to this word list, so
    # a question word missing from here simply cannot be transcribed -- which
    # would silently make whole question types impossible to ask.
    'how many much far close where is are there any anyone anybody someone '
    'somebody person people persons distance from away room here around '
    'which tell me about count look check find out again now tall big '
    # Ordinary function words. A word absent from this list CANNOT be
    # transcribed at all, so a missing "in" quietly mangles every "...in the
    # room" question. Cheap to include and they cannot form a command on
    # their own -- the wake phrase and intent match still gate everything.
    'in on at of and or it this that your for with anything everything '
    'nearby next left right front behind side nothing else other'
).split()


class VoiceCommand(Node):

    def __init__(self):
        super().__init__('qcar2_voice_command')
        share = get_package_share_directory('qcar2_rviz_gui')
        project_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz')

        self.declare_parameter('model_path',
                               os.path.join(project_dir, 'models', 'vosk-model-small-en-us-0.15'))
        self.declare_parameter('vocabulary_file',
                               os.path.join(share, 'config', 'object_vocabulary.yaml'))
        self.declare_parameter('wake_phrase', 'hey car')
        self.declare_parameter('arm_seconds', 6.0)
        self.declare_parameter('source', 'car')
        self.declare_parameter('device', '')
        self.declare_parameter('emergency_stop_without_wake', True)
        self.declare_parameter('restrict_grammar', True)
        self.declare_parameter('enabled', False)

        g = lambda n: self.get_parameter(n).value
        self.model_path = g('model_path')
        self.wake = str(g('wake_phrase')).lower().strip()
        self.arm_seconds = float(g('arm_seconds'))
        self.source = str(g('source')).lower()
        self.device = str(g('device')) or None
        self.bare_stop = bool(g('emergency_stop_without_wake'))
        self.restrict = bool(g('restrict_grammar'))

        self.words = self.build_grammar(g('vocabulary_file'))

        self.enabled = bool(g('enabled'))
        self.armed_until = 0.0
        self.status = 'idle'
        self.last_action = ''
        self.last_final = ''
        self.partial = ''
        self.error = ''

        # Half-duplex: audio is discarded until this monotonic time. Set while
        # the car's own speaker is talking (see on_speaking) so the mic cannot
        # hear -- and act on -- the car's own voice.
        self.muted_until = 0.0
        self.flush_pending = False
        # What was heard and what was done with it, newest first, for the
        # console's "Heard" card. Recording the DECISION, not just the words,
        # is what makes it useful: "ignored -- no 'hey car'" tells you the mic
        # works and the gate is doing its job, which a bare transcript cannot.
        self.history = deque(maxlen=8)
        self.audio = queue.Queue(maxsize=64)
        self.lock = threading.Lock()
        self.stream = None
        self.recognizer = None
        self.model = None

        self.nav_pub = self.create_publisher(String, '/qcar2/nav_to_object', 10)
        self.ask_pub = self.create_publisher(String, '/qcar2/ask', 10)
        self.say_pub = self.create_publisher(String, '/qcar2/say', 10)
        self.estop_pub = self.create_publisher(Bool, '/qcar2_estop', LATCHED)
        self.transcript_pub = self.create_publisher(String, '/qcar2/voice_transcript', 10)
        self.cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_subscription(Bool, '/qcar2/mic_enabled', self.on_enabled, LATCHED)
        self.create_subscription(String, '/qcar2/mic_source', self.on_source, LATCHED)
        self.create_subscription(UInt8MultiArray, '/qcar2/mic_audio', self.on_browser_audio, 10)
        self.create_subscription(Bool, '/qcar2/speaking', self.on_speaking, 10)

        self.running = True
        threading.Thread(target=self.recognize_loop, daemon=True).start()
        self.create_timer(0.3, self.publish_transcript)

        self.get_logger().info(
            f'Voice ready (OFF until enabled). Wake phrase: "{self.wake}", source: {self.source}')

    # ------------------------------------------------------------- grammar

    def build_grammar(self, vocab_path):
        """Word list the recogniser is restricted to.

        Built from the SAME vocabulary YAML the detector uses, so an object you
        add there becomes speakable without touching this file.
        """
        words = set(BASE_WORDS)
        words.update(self.wake.split())
        try:
            with open(vocab_path) as fh:
                data = yaml.safe_load(fh) or {}
        except OSError as exc:
            self.get_logger().warn(f'Cannot read vocabulary {vocab_path}: {exc}')
            return sorted(words)
        for label in data.get('classes') or []:
            words.update(str(label).lower().split())
            # Plurals too: people say "how many chairs", and a word the
            # grammar does not contain simply cannot come out of the
            # recogniser.
            for tok in str(label).lower().split():
                words.add(tok + 's' if not tok.endswith('s') else tok)
        for canonical, aliases in (data.get('synonyms') or {}).items():
            words.update(str(canonical).lower().split())
            for alias in aliases or []:
                words.update(str(alias).lower().split())
        # Vosk's small model has a fixed lexicon; a word outside it makes the
        # whole grammar fail to compile. Keep plain alphabetic tokens only.
        return sorted(w for w in words if w and w.isalpha())

    # -------------------------------------------------------------- control

    def on_enabled(self, msg):
        want = bool(msg.data)
        if want == self.enabled:
            return
        self.enabled = want
        self.get_logger().info(f'Microphone {"ENABLED" if want else "disabled"}')
        if want:
            self.start_audio()
        else:
            self.stop_audio()
            self.armed_until = 0.0
            self.status = 'idle'
            self.partial = ''

    def on_source(self, msg):
        source = (msg.data or '').lower().strip()
        if source not in ('car', 'browser') or source == self.source:
            return
        self.get_logger().info(f'Microphone source -> {source}')
        self.stop_audio()
        self.source = source
        with self.lock:
            self.recognizer = None      # drop half-decoded audio from the old source
        if self.enabled:
            self.start_audio()

    # ---------------------------------------------------------------- audio

    def start_audio(self):
        if self.source != 'car':
            self.status = 'listening (browser)'
            return
        if self.stream is not None:
            return
        try:
            import sounddevice as sd
        except ImportError:
            self.error = 'sounddevice not installed; run scripts/install_deps.sh'
            self.get_logger().error(self.error)
            self.status = 'error'
            return

        def callback(indata, _frames, _t, status):
            if status:
                self.get_logger().debug(f'audio status: {status}')
            try:
                self.audio.put_nowait(bytes(indata))
            except queue.Full:
                pass                     # drop rather than lag behind the speaker

        try:
            self.stream = sd.RawInputStream(
                samplerate=SAMPLE_RATE, blocksize=4000, dtype='int16',
                channels=1, device=self.device, callback=callback)
            self.stream.start()
            self.status = 'listening'
            self.error = ''
        except Exception as exc:         # noqa: BLE001 - surface to the console
            self.stream = None
            self.error = f'Cannot open the car microphone: {exc}'
            self.get_logger().error(self.error)
            self.status = 'error'

    def stop_audio(self):
        if self.stream is not None:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception:            # noqa: BLE001 - shutting down anyway
                pass
            self.stream = None
        while not self.audio.empty():
            try:
                self.audio.get_nowait()
            except queue.Empty:
                break

    # Seconds to stay deaf AFTER the speaker says it has finished: covers the
    # PulseAudio output buffer still draining and the room's reverb tail.
    SPEECH_TAIL_SEC = 0.6
    # Hard ceiling on one mute. If the announcer died mid-sentence and never
    # sent the False edge, the mic must not stay deaf forever.
    MAX_MUTE_SEC = 20.0

    def on_speaking(self, msg):
        now = time.monotonic()
        if msg.data:
            self.muted_until = now + self.MAX_MUTE_SEC
        else:
            self.muted_until = now + self.SPEECH_TAIL_SEC
            # If we were waiting for a command (after "hey car" -> "Yes?"),
            # start the listening window from when the car STOPPED talking,
            # so its own prompt does not eat into the user's time to answer.
            if self.armed_until > now:
                self.armed_until = self.muted_until + self.arm_seconds
            # Whatever the recogniser half-heard during the car's speech is
            # the car's own voice; throw it away rather than let it be glued
            # onto the start of the user's next sentence.
            self.flush_pending = True

    def muted(self):
        return time.monotonic() < self.muted_until

    def on_browser_audio(self, msg):
        if not self.enabled or self.source != 'browser':
            return
        try:
            self.audio.put_nowait(bytes(msg.data))
        except queue.Full:
            pass

    # ---------------------------------------------------------- recognition

    def ensure_recognizer(self):
        if self.recognizer is not None:
            return self.recognizer
        try:
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError:
            self.error = 'vosk not installed; run scripts/install_deps.sh'
            self.status = 'error'
            return None
        if not os.path.isdir(self.model_path):
            self.error = f'Vosk model missing at {self.model_path}; run scripts/install_deps.sh'
            self.status = 'error'
            return None
        try:
            SetLogLevel(-1)
            if self.model is None:
                self.get_logger().info(f'Loading speech model {self.model_path} ...')
                self.model = Model(self.model_path)
            if self.restrict:
                grammar = json.dumps(self.words + ['[unk]'])
                rec = KaldiRecognizer(self.model, SAMPLE_RATE, grammar)
            else:
                rec = KaldiRecognizer(self.model, SAMPLE_RATE)
            rec.SetWords(False)
            self.recognizer = rec
            self.error = ''
            self.get_logger().info(
                f'Speech recogniser ready ({len(self.words)} words in grammar)'
                if self.restrict else 'Speech recogniser ready (open vocabulary)')
        except Exception as exc:         # noqa: BLE001 - report, keep node alive
            self.error = f'Speech recogniser failed: {exc}'
            self.get_logger().error(self.error)
            self.status = 'error'
            return None
        return self.recognizer

    def recognize_loop(self):
        while self.running and rclpy.ok():
            if not self.enabled:
                time.sleep(0.2)
                continue
            rec = self.ensure_recognizer()
            if rec is None:
                time.sleep(3.0)
                continue
            try:
                chunk = self.audio.get(timeout=0.3)
            except queue.Empty:
                continue
            if self.muted():
                continue                  # the car is talking; not the user
            if self.flush_pending:
                self.flush_pending = False
                try:
                    rec.Reset()
                except Exception:         # noqa: BLE001 - older vosk: rebuild
                    self.recognizer = None
                    continue
                self.partial = ''
            try:
                if rec.AcceptWaveform(chunk):
                    text = json.loads(rec.Result()).get('text', '')
                    if text.strip():
                        self.partial = ''
                        self.on_utterance(text.strip().lower())
                else:
                    self.partial = json.loads(rec.PartialResult()).get('partial', '')
            except Exception as exc:     # noqa: BLE001 - never kill the thread
                self.get_logger().warn(f'Recognition error: {exc}')

    # ------------------------------------------------------------- intents

    def on_utterance(self, text):
        raw = text
        text = re.sub(r'\[unk\]', ' ', text)
        text = re.sub(r'\s+', ' ', text).strip()
        if not text:
            # Speech was heard but none of it was in the vocabulary. Still
            # worth showing: it proves the mic is picking you up, and says
            # why nothing happened.
            if '[unk]' in raw:
                self.log_heard('(speech I do not have words for)', 'ignored')
            return
        self.last_final = text
        self.get_logger().info(f'heard: "{text}"')
        self.result = 'ignored -- no "hey car"'
        try:
            return self._decide(text)
        finally:
            self.log_heard(text, self.result)

    def log_heard(self, text, result):
        # Collapse a run of unrecognised speech into one counted line instead
        # of letting background chatter push every real command off the card.
        if self.history and self.history[0]['text'] == text == '(speech I do not have words for)':
            self.history[0]['n'] = self.history[0].get('n', 1) + 1
            self.history[0]['t'] = time.time()
            return
        self.history.appendleft({'text': text, 'result': result,
                                 't': round(time.time(), 1)})

    def _decide(self, text):

        # Bare emergency stop, wake phrase or not: a false stop is cheap, a
        # missed one is not.
        if self.bare_stop and self.is_command(text, STOP_WORDS):
            return self.do_stop()

        armed = time.monotonic() < self.armed_until
        if self.wake and self.wake in text:
            remainder = text.split(self.wake, 1)[1].strip()
            if not remainder:
                # Wake phrase alone: open a short window for the command.
                self.armed_until = time.monotonic() + self.arm_seconds
                self.status = 'armed'
                self.result = 'heard "hey car" -- say your command'
                self.say('Yes?')
                return
            self.armed_until = 0.0
            return self.execute(remainder)
        if armed:
            self.armed_until = 0.0
            return self.execute(text)
        # No wake phrase and not armed -> this was conversation, not a command.
        self.status = 'listening'

    @staticmethod
    def is_command(text, words):
        return any(re.search(rf'\b{re.escape(w)}\b', text) for w in words)

    def execute(self, text):
        self.status = 'listening'
        if self.is_command(text, STOP_WORDS):
            return self.do_stop()
        if self.is_command(text, CANCEL_WORDS):
            return self.do_cancel()
        if self.is_command(text, RESUME_WORDS):
            return self.do_resume()
        # Questions go to the assistant. Must be tested BEFORE the go-to
        # pattern: "how far is the sofa" contains an object name and would
        # otherwise be obeyed as "drive to the sofa".
        if QUESTION_PATTERNS.search(text) or self.is_command(text, LIST_WORDS):
            msg = String()
            msg.data = text
            self.ask_pub.publish(msg)
            self.act('ask', f'question: {text}')
            return
        match = GO_PATTERNS.search(text)
        target = (match.group(3) if match else text).strip()
        # \b (not \s+) so a trailing bare article is stripped too: "go to the"
        # must re-prompt, not send the literal object name "the".
        target = re.sub(r'^(the|a|an|my)\b\s*', '', target).strip()
        if not target:
            self.result = 'no destination heard -- asked "where should I go?"'
            self.say("Where should I go?")
            self.armed_until = time.monotonic() + self.arm_seconds
            self.status = 'armed'
            return
        # Resolution and the "I don't know that object" reply are qcar2_object_nav's
        # job -- it owns the map's landmark list, so the answer stays correct
        # even as objects are added during a mapping run.
        msg = String()
        msg.data = target
        self.nav_pub.publish(msg)
        self.act('go', f'go to: {target}')

    def do_stop(self):
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)
        self.cancel()
        self.act('stop', 'Stopping')
        self.say('Stopping')

    def do_cancel(self):
        self.cancel()
        self.act('cancel', 'Goal cancelled')
        self.say('Cancelled')

    def do_resume(self):
        msg = Bool()
        msg.data = False
        self.estop_pub.publish(msg)
        self.act('resume', 'Emergency stop released')
        self.say('Released')

    def cancel(self):
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(CancelGoal.Request())

    def act(self, kind, detail):
        self.last_action = detail
        self.result = detail
        self.get_logger().info(f'action [{kind}]: {detail}')

    def say(self, text):
        msg = String()
        msg.data = text
        self.say_pub.publish(msg)

    # -------------------------------------------------------------- outputs

    def publish_transcript(self):
        if time.monotonic() >= self.armed_until and self.status == 'armed':
            self.status = 'listening'
        msg = String()
        msg.data = json.dumps({
            'enabled': self.enabled,
            'source': self.source,
            'status': self.status if self.enabled else 'off',
            'partial': self.partial,
            'final': self.last_final,
            'action': self.last_action,
            'armed': time.monotonic() < self.armed_until,
            'wake': self.wake,
            'history': list(self.history),
            'error': self.error,
        })
        self.transcript_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.stop_audio()
        super().destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VoiceCommand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
