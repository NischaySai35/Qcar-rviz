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

TWO RECOGNISERS (2026-10-05)
----------------------------
The restricted Vosk grammar is great at "hey car" and "stop" but mangles
everything else ("how many sofas are there" came out "how many sofa date"),
and it was a US-English model. So:
  * Vosk -- now the Indian English small model -- stays always-on and only
    decides WHEN: wake phrase, bare stop, sentence boundaries.
  * Whisper small.en (whisper.cpp server on the GPU, 127.0.0.1:8091, started
    by navigate.launch.py) decides WHAT: once a sentence with "hey car" (or
    one inside the "Yes?" window) finishes, that sentence's audio is
    transcribed by Whisper (~0.3 s warm) and THAT text is acted on. No word
    list, so any phrasing works.
If the Whisper server is not running or fails, the Vosk text is used exactly
as before, so voice never goes deaf.

TWO MICROPHONES
---------------
  car      -- the QCar2's two onboard mics (a PDM pair on the Orin's DMIC2
              port), read through PulseAudio -- preferably the echo-cancelled
              copy qcar2_announcer.py sets up, so the car cannot hear its own
              voice. 48 kHz stereo, cleaned up by MicFrontEnd (hum filter,
              gain) before Vosk. Needs no browser tab open, but works best
              when you speak clearly and fairly near the car.
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

import io
import json
import math
import os
import queue
from collections import deque
import re
import subprocess
import threading
import time
import urllib.request
import uuid
import wave

# The car mic is read through PulseAudio, which is found via XDG_RUNTIME_DIR;
# an xrdp/SSH-started launch lacks it. Same fix and story as
# qcar2_announcer.py (the speaker was silent for exactly this reason).
if not os.environ.get('XDG_RUNTIME_DIR') and os.path.isdir(f'/run/user/{os.getuid()}'):
    os.environ['XDG_RUNTIME_DIR'] = f'/run/user/{os.getuid()}'

import numpy as np
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

# The car's microphones, in order of preference: the echo-cancelled source
# qcar2_announcer.py sets up (the car's own voice already subtracted), then
# the raw DMIC2 pair. PulseAudio's DEFAULT source is neither -- it is the
# I2S1 capture side, which has nothing wired to it and records a flat
# constant. That, not the grammar or the wake word, is why the car mic
# never heard anything.
CAR_MIC_SOURCES = ('qcar2_mic_ec', 'alsa_input.hw_1_1')
CAR_MIC_RATE = 48000

# Seconds of silence after the recogniser closes an utterance before it is
# acted on. Vosk ends an utterance at ~0.5 s of silence, and people pause
# longer than that mid-sentence ("hey car ... go to the sofa") -- so its
# utterances are PIECES of what was said, and acting on each piece split
# every command in two. Pieces closer together than this are one sentence.
JOIN_SEC = 1.0

# Audio kept for Whisper: enough history to cover one long sentence plus the
# pre-roll, never more (it is 32 kB per second).
RING_SEC = 20.0
# Audio taken from BEFORE the recogniser first reported speech: Vosk's first
# partial arrives a few hundred ms into a word, and Whisper needs the onset.
PREROLL_SEC = 0.6


def normalize_whisper(text):
    """Whisper writes "Go to sofa 2." -- the intent code wants "go to sofa 2"."""
    text = (text or '').lower().replace('’', "'")
    text = re.sub(r'\[[^\]]*\]|\([^)]*\)', ' ', text)   # [BLANK_AUDIO], (music)
    # Expand before dropping apostrophes: "where's" -> "wheres" matched none
    # of the assistant's question rules.
    text = re.sub(r"\b(what|where|who|how|that|there|it|here)'s\b", r'\1 is', text)
    text = re.sub(r"'re\b", ' are', text)
    text = re.sub(r"[^a-z0-9'\s]", ' ', text).replace("'", '')
    return re.sub(r'\s+', ' ', text).strip()


class MicFrontEnd:
    """Car DMIC audio (48 kHz, 1-2 channels, float) -> what Vosk needs.

    Measured on this car with a real recorded human voice played through the
    car speaker and captured on DMIC2:

      * Both mics summed:        best word error (0.27 vs 0.33 for one mic).
      * High-pass at 100 Hz:     ESSENTIAL. Below ~150 Hz the mic is swamped
        by DC drift and fan/motor hum (0-60 Hz band louder than the speech
        itself); unfiltered, Vosk transcribed NOTHING at any volume. Above
        600 Hz the speech sits 12-24 dB over the noise. Band-limiting harder
        (200-7000 Hz) was no better, so the gentlest filter that works wins.
      * Gain:                    speech arrives near -65 dBFS. Converted to
        int16 as-is that is a few LSBs -- unusable -- so the signal is lifted
        until the noise floor sits at `floor_dbfs`, with normal speech 15-25
        dB above it. Tracking the FLOOR (not the speech peaks) keeps the gain
        steady while someone talks instead of pumping.

    Range: recognition falls off steeply once speech is ~12-18 dB quieter than
    the test recording (played on the car's own speaker, centimetres from the
    mics). How far away a real person can stand is not measured -- speak
    clearly and near the car, or use the browser mic from further away.

    Stateful across blocks (filter state, decimation phase), so a stream fed
    in chunks is identical to the same audio processed in one go -- no
    clicks at block edges for the recogniser to hear as consonants.
    """

    def __init__(self, in_rate=CAR_MIC_RATE, out_rate=SAMPLE_RATE,
                 floor_dbfs=-35.0, max_gain_db=60.0):
        # -35 dBFS measured best on real speech through the car mic (word
        # error 0.13 / 0.53 / 0.73 at -6 / -12 / -18 dB playback, vs 0.27 /
        # 0.53 / 1.00 at -45), with no clipping.
        from scipy.signal import butter, firwin
        if in_rate % out_rate:
            raise ValueError('input rate must be a multiple of the output rate')
        self.decim = in_rate // out_rate
        self.hp = butter(2, 100, 'highpass', fs=in_rate, output='sos')
        # Filter state is primed from the first sample (see process()): the
        # mic sits at a ~2 % DC offset, and starting from zero state turns
        # that into a step the high-pass rings on for a second or more --
        # which, before this, ate the first words after every mic enable.
        self.hp_zi = None
        # Anti-alias before keeping every 3rd sample: everything above 7 kHz
        # would otherwise fold back into the speech band as hiss.
        self.lp = firwin(63, 7000, fs=in_rate)
        self.lp_zi = np.zeros(len(self.lp) - 1)
        self.n_in = 0
        self.floor_target = 10 ** (floor_dbfs / 20)
        self.max_gain = 10 ** (max_gain_db / 20)
        self.floor = None
        self.gain = 1.0

    def process(self, block):
        """float array (n,) or (n, channels) at in_rate -> int16 bytes at 16 kHz."""
        from scipy.signal import lfilter, sosfilt, sosfilt_zi
        x = np.asarray(block, dtype=np.float64)
        if x.ndim == 2:
            x = x.sum(axis=1)
        if not len(x):
            return b''
        if self.hp_zi is None:
            self.hp_zi = sosfilt_zi(self.hp) * x[0]      # as if it had always been at this level
        x, self.hp_zi = sosfilt(self.hp, x, zi=self.hp_zi)
        y, self.lp_zi = lfilter(self.lp, 1.0, x, zi=self.lp_zi)
        start = (-self.n_in) % self.decim
        self.n_in += len(y)
        y = y[start::self.decim]
        if not len(y):
            return b''
        rms = float(np.sqrt(np.mean(y * y))) + 1e-12
        # Floor: drops quickly to any quieter block, creeps up slowly (so a
        # sentence does not drag it up, but a louder room eventually does).
        if self.floor is None or rms < self.floor:
            self.floor = rms if self.floor is None else 0.4 * self.floor + 0.6 * rms
        else:
            self.floor *= 1.01
        want = min(self.max_gain, max(1.0, self.floor_target / self.floor))
        self.gain += 0.2 * (want - self.gain)
        return (np.clip(y * self.gain, -1.0, 1.0) * 32767).astype(np.int16).tobytes()

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
    r'what do you|which|anyone|anybody|distance to|'
    # Time, camera and patrol requests. "around"/"patrol" must be here: this
    # check runs BEFORE the go-to pattern, and "go around the room and tell
    # me how many people are near the sofa" would otherwise drive to the sofa.
    r'what time|the date|what day|describe|what colou?r|tell me|around|patrol|'
    r'explore|who|why|when)\b', re.I)

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
    'nearby next left right front behind side nothing else other '
    # Assistant questions answered by the clock and the vision model
    # (qcar2_assistant.py). Interim until Whisper takes over the sentence
    # after "hey car" -- until then a question can only use listed words.
    'time date day today describe colour color patrol explore whole '
    'who why when doing joke yourself room scene '
    # Numbered duplicates: "go to sofa two", "where is the second armchair".
    'number one two three four five six seven eight nine ten '
    'first second third fourth fifth sixth seventh eighth ninth tenth'
).split()


class VoiceCommand(Node):

    def __init__(self):
        super().__init__('qcar2_voice_command')
        share = get_package_share_directory('qcar2_rviz_gui')
        project_dir = os.path.join(os.path.expanduser('~'), 'Desktop', 'Qcar-rviz')

        # Indian English (was vosk-model-small-en-us-0.15). Used only for the
        # wake phrase, bare stop and sentence boundaries -- see TWO RECOGNISERS.
        self.declare_parameter('model_path',
                               os.path.join(project_dir, 'models', 'vosk-model-small-en-in-0.4'))
        self.declare_parameter('use_whisper', True)
        self.declare_parameter('whisper_url', 'http://127.0.0.1:8091')
        self.declare_parameter('whisper_timeout_sec', 8.0)
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
        self.use_whisper = bool(g('use_whisper'))
        self.whisper_url = str(g('whisper_url')).rstrip('/')
        self.whisper_timeout = float(g('whisper_timeout_sec'))
        self.whisper_ok = False        # set once the server answered the warm-up

        self.words = self.build_grammar(g('vocabulary_file'))
        # Whisper's initial prompt: biases it toward this room's object names
        # ("armchair", "air cooler") and the command style, which cuts errors
        # on exactly the words that matter.
        self.whisper_prompt = (f'{self.wake}, go to the sofa 2. How many armchairs are there? '
                               + ', '.join(self.object_names[:40]) + '.')

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
        # Recogniser utterances not yet acted on -- pieces of one sentence,
        # joined once JOIN_SEC of silence says the speaker has finished.
        self.pieces = []
        self.last_speech = 0.0
        # Recent 16 kHz audio as (arrival time, bytes), and when the sentence
        # being built started -- what is handed to Whisper on finish.
        self.ring = deque()
        self.sentence_start = None
        self.pending_audio = b''
        # Set when Whisper re-transcribed the sentence; the Heard card then
        # shows its text (what was acted on), not the rough Vosk one.
        self.heard_text = None
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
        if self.use_whisper:
            threading.Thread(target=self.warm_whisper, daemon=True).start()
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
        self.object_names = []
        try:
            with open(vocab_path) as fh:
                data = yaml.safe_load(fh) or {}
        except OSError as exc:
            self.get_logger().warn(f'Cannot read vocabulary {vocab_path}: {exc}')
            return sorted(words)
        self.object_names = [str(c).lower() for c in data.get('classes') or []]
        for label in data.get('classes') or []:
            toks = str(label).lower().split()
            words.update(toks)
            # Plurals too: people say "how many chairs", and a word the
            # grammar does not contain simply cannot come out of the
            # recogniser. Only the LAST word takes the plural ("dining
            # tables", not "dinings tables") -- the others were words Vosk
            # does not know, logged as a warning on every start.
            if toks:
                words.add(self.plural(toks[-1]))
        for canonical, aliases in (data.get('synonyms') or {}).items():
            words.update(str(canonical).lower().split())
            for alias in aliases or []:
                words.update(str(alias).lower().split())
        # Vosk's small model has a fixed lexicon; a word outside it makes the
        # whole grammar fail to compile. Keep plain alphabetic tokens only.
        return sorted(w for w in words if w and w.isalpha())

    @staticmethod
    def plural(word):
        if word.endswith(('s', 'sh', 'ch', 'x')):
            return word if word.endswith('s') else word + 'es'
        if word.endswith('f'):
            return word[:-1] + 'ves'          # bookshelf -> bookshelves
        if word.endswith('y') and word[-2:-1] not in 'aeiou':
            return word[:-1] + 'ies'
        return word + 's'

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
            self.pieces = []

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
            self.error = 'sounddevice not installed; see "Setup" in README.md'
            self.get_logger().error(self.error)
            self.status = 'error'
            return

        pulse_source = None if self.device else self.pick_car_mic()
        front_end = MicFrontEnd() if pulse_source else None

        def callback(indata, _frames, _t, status):
            if status:
                self.get_logger().debug(f'audio status: {status}')
            data = bytes(indata)
            if front_end is not None:
                data = front_end.process(np.frombuffer(data, dtype=np.float32).reshape(-1, 2))
            try:
                self.audio.put_nowait(data)
            except queue.Full:
                pass                     # drop rather than lag behind the speaker

        try:
            if pulse_source:
                # The ALSA "pulse" device opens whatever PULSE_SOURCE names;
                # it is read when the stream connects, so set it right here.
                os.environ['PULSE_SOURCE'] = pulse_source
                self.stream = sd.RawInputStream(
                    samplerate=CAR_MIC_RATE, blocksize=CAR_MIC_RATE // 10, dtype='float32',
                    channels=2, device='pulse', callback=callback)
            else:
                # An explicit `device` parameter (or no PulseAudio car mic):
                # the old direct path, 16 kHz int16 straight to Vosk.
                self.stream = sd.RawInputStream(
                    samplerate=SAMPLE_RATE, blocksize=4000, dtype='int16',
                    channels=1, device=self.device, callback=callback)
            self.stream.start()
            self.status = 'listening'
            self.error = ''
            self.get_logger().info(
                f'Car microphone open: {pulse_source or self.device or "default input"}'
                + (' (echo-cancelled)' if pulse_source == CAR_MIC_SOURCES[0] else ''))
        except Exception as exc:         # noqa: BLE001 - surface to the console
            self.stream = None
            self.error = f'Cannot open the car microphone: {exc}'
            self.get_logger().error(self.error)
            self.status = 'error'

    def pick_car_mic(self):
        """PulseAudio source for the car's microphones, or None if absent."""
        try:
            out = subprocess.run(['pactl', 'list', 'short', 'sources'], check=False,
                                 timeout=3, capture_output=True, text=True).stdout
        except (OSError, subprocess.TimeoutExpired):
            return None
        names = {line.split('\t')[1] for line in out.splitlines() if '\t' in line}
        for name in CAR_MIC_SOURCES:
            if name in names:
                return name
        self.get_logger().warn(
            f'None of {CAR_MIC_SOURCES} exist in PulseAudio; falling back to the default '
            'input, which on this car is the silent I2S1 capture.')
        return None

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
            self.error = 'vosk not installed; see "Setup" in README.md'
            self.status = 'error'
            return None
        if not os.path.isdir(self.model_path):
            self.error = f'Vosk model missing at {self.model_path}; see "Setup" in README.md'
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
            # Short timeout: the join deadline below is checked on every
            # pass, so this bounds how late a finished sentence is acted on.
            try:
                chunk = self.audio.get(timeout=0.1)
            except queue.Empty:
                self.maybe_finish_sentence()
                continue
            if self.muted():
                self.maybe_finish_sentence()
                continue                  # the car is talking; not the user
            if self.flush_pending:
                self.flush_pending = False
                try:
                    rec.Reset()
                except Exception:         # noqa: BLE001 - older vosk: rebuild
                    self.recognizer = None
                    continue
                self.partial = ''
                self.ring.clear()          # the car's own voice; never send it on
                self.sentence_start = None
            now = time.monotonic()
            self.ring.append((now, chunk))
            while self.ring and now - self.ring[0][0] > RING_SEC:
                self.ring.popleft()
            try:
                if rec.AcceptWaveform(chunk):
                    text = json.loads(rec.Result()).get('text', '')
                    self.partial = ''
                    if text.strip():
                        self.add_piece(text.strip().lower())
                else:
                    self.partial = json.loads(rec.PartialResult()).get('partial', '')
                    if self.partial:
                        self.last_speech = time.monotonic()   # still talking
                        if self.sentence_start is None:
                            # First word of a new sentence: mark where its
                            # audio begins (minus this chunk and a pre-roll).
                            self.sentence_start = now - self.chunk_sec(chunk) - PREROLL_SEC
            except Exception as exc:     # noqa: BLE001 - never kill the thread
                self.get_logger().warn(f'Recognition error: {exc}')
            self.maybe_finish_sentence()

    @staticmethod
    def chunk_sec(chunk):
        return len(chunk) / 2.0 / SAMPLE_RATE      # int16 mono

    def sentence_audio(self):
        """This sentence's 16 kHz int16 audio (bytes) from the ring, or b''."""
        start = self.sentence_start
        self.sentence_start = None
        if start is None:
            return b''
        return b''.join(c for t, c in self.ring if t >= start)

    def add_piece(self, text):
        """One recogniser utterance: hold it until the sentence is finished.

        Exception: a bare stop word is acted on at once, together with
        whatever was already held -- an emergency stop must not wait out
        the join window.
        """
        self.pieces.append(text)
        self.last_speech = time.monotonic()
        if self.sentence_start is None:
            # A short word can close before any partial was reported.
            self.sentence_start = self.last_speech - 1.5 - PREROLL_SEC
        if self.bare_stop and self.is_command(text, STOP_WORDS):
            self.finish_sentence()

    def maybe_finish_sentence(self, now=None):
        """Act on the held pieces once nobody has spoken for JOIN_SEC."""
        now = time.monotonic() if now is None else now
        if self.pieces and not self.partial and now - self.last_speech >= JOIN_SEC:
            self.finish_sentence()

    def finish_sentence(self):
        text = ' '.join(self.pieces)
        self.pieces = []
        self.pending_audio = self.sentence_audio()
        self.on_utterance(text)

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
        self.heard_text = None
        try:
            return self._decide(text)
        finally:
            if self.heard_text:
                self.last_final = self.heard_text
            self.log_heard(self.heard_text or text, self.result)

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
        # Tolerant wake match. "hey" is the fragile word -- in testing the
        # Indian model returned just "car" for a clear "hey car" -- so a
        # sentence that STARTS with "car" (optionally after hey/hi/a) counts
        # too. "car" is only in the grammar because of the wake phrase, so
        # ordinary conversation cannot produce it at the start of a sentence.
        wake_hit = None
        if self.wake and self.wake in text:
            wake_hit = text.split(self.wake, 1)[1]
        elif self.wake == 'hey car':
            m = re.match(r'^(?:(?:hey|hi|a)\s+)?car\b(.*)$', text)
            if m:
                wake_hit = m.group(1)
        if wake_hit is not None:
            remainder = wake_hit.strip()
            if not remainder:
                # Wake phrase alone: open a short window for the command.
                self.armed_until = time.monotonic() + self.arm_seconds
                self.status = 'armed'
                self.result = 'heard "hey car" -- say your command'
                self.say('Yes?')
                return
            self.armed_until = 0.0
            return self.execute(self.refine(remainder, strip_wake=True))
        if armed:
            self.armed_until = 0.0
            return self.execute(self.refine(text, strip_wake=False))
        # No wake phrase and not armed -> this was conversation, not a command.
        self.status = 'listening'

    # ------------------------------------------------------------- whisper

    def refine(self, vosk_text, strip_wake):
        """The sentence to act on: Whisper's transcript of this sentence's
        audio when available, else the Vosk text (the old behaviour)."""
        audio, self.pending_audio = self.pending_audio, b''
        if not (self.use_whisper and self.whisper_ok) or len(audio) < SAMPLE_RATE // 4:
            return vosk_text
        t0 = time.monotonic()
        text = normalize_whisper(self.transcribe(audio))
        if strip_wake:
            # Whisper writes the wake phrase however it heard it ("hey car",
            # "hey, car", "a car", or just "car"); drop it from the start.
            text = re.sub(r'^(?:(?:hey|hi|hay|a|okay|ok)\s+)?(?:car|cars|kar)\b\s*', '', text,
                          count=1)
        if not text:
            return vosk_text
        self.get_logger().info(f'whisper ({time.monotonic() - t0:.2f}s): "{text}" '
                               f'(vosk had "{vosk_text}")')
        self.heard_text = text
        return text

    def transcribe(self, pcm):
        """16 kHz int16 mono bytes -> text via whisper.cpp's /inference."""
        buf = io.BytesIO()
        with wave.open(buf, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SAMPLE_RATE)
            w.writeframes(pcm)
        boundary = uuid.uuid4().hex
        fields = {'response_format': 'json', 'temperature': '0.0',
                  'prompt': self.whisper_prompt}
        body = b''.join(
            f'--{boundary}\r\nContent-Disposition: form-data; name="{k}"\r\n\r\n{v}\r\n'.encode()
            for k, v in fields.items())
        body += (f'--{boundary}\r\nContent-Disposition: form-data; name="file"; '
                 f'filename="s.wav"\r\nContent-Type: audio/wav\r\n\r\n').encode()
        body += buf.getvalue() + f'\r\n--{boundary}--\r\n'.encode()
        req = urllib.request.Request(
            self.whisper_url + '/inference', data=body,
            headers={'Content-Type': f'multipart/form-data; boundary={boundary}'})
        try:
            with urllib.request.urlopen(req, timeout=self.whisper_timeout) as resp:
                return json.loads(resp.read().decode()).get('text', '')
        except Exception as exc:          # noqa: BLE001 - fall back to Vosk
            self.get_logger().warn(f'Whisper failed ({exc}); using the Vosk text.')
            return ''

    def warm_whisper(self):
        """Wait for the server (started by the same launch), then pay its
        ~1.3 s first-request cost on silence instead of a real command."""
        deadline = time.monotonic() + 120.0
        while self.running and time.monotonic() < deadline:
            try:
                urllib.request.urlopen(self.whisper_url + '/', timeout=2.0).close()
                break
            except Exception:             # noqa: BLE001 - not up yet
                time.sleep(1.0)
        else:
            self.get_logger().info('No Whisper server; voice uses Vosk only.')
            return
        self.transcribe(b'\x00\x00' * SAMPLE_RATE)
        self.whisper_ok = True
        self.get_logger().info('Whisper ready: sentences after "hey car" are transcribed by it.')

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
        # No "go to": a bare object name ("hey car, sofa") is still a
        # destination, but anything longer is a sentence for the assistant
        # (which also has the vision model) rather than an object called
        # "can you say something".
        if not match and len(text.split()) > 3:
            msg = String()
            msg.data = text
            self.ask_pub.publish(msg)
            self.act('ask', f'question: {text}')
            return
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
            # The sentence as it builds up: pieces already closed by the
            # recogniser plus the words still being decoded, so the Heard
            # card shows ONE growing line instead of fragments.
            'partial': ' '.join(self.pieces + ([self.partial] if self.partial else [])).replace(
                '[unk]', '').strip(),
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
