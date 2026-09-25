#!/usr/bin/python3
"""Offline checks for qcar2_voice_command intent gating.

Run with scripts/run_tests.sh. Imports the node module but never constructs a
Node and never calls rclpy.init().

The headline requirement: with the microphone ON, ordinary conversation in the
room must NOT move the car.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qcar2_ws', 'src', 'qcar2_rviz_gui', 'scripts'))
import qcar2_voice_command as vc  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


class Fake(vc.VoiceCommand):
    """Real intent logic, fake I/O."""

    def __init__(self, bare_stop=True):
        self.wake = 'hey car'
        self.arm_seconds = 6.0
        self.bare_stop = bare_stop
        self.armed_until = 0.0
        self.status = 'listening'
        self.last_action = ''
        self.last_final = ''
        self.events = []
        from collections import deque
        self.history = deque(maxlen=8)

    def get_logger(self):
        class L:
            def info(self, *_a, **_k): pass
            def warn(self, *_a, **_k): pass
            def debug(self, *_a, **_k): pass
        return L()

    def say(self, text):
        self.events.append(('say', text))

    def cancel(self):
        self.events.append(('cancel_srv', None))

    class _Pub:
        def __init__(self, sink, kind):
            self.sink, self.kind = sink, kind

        def publish(self, msg):
            self.sink.append((self.kind, getattr(msg, 'data', None)))

    @property
    def nav_pub(self):
        return Fake._Pub(self.events, 'GOAL')

    @property
    def estop_pub(self):
        return Fake._Pub(self.events, 'ESTOP')

    def goals(self):
        return [v for k, v in self.events if k == 'GOAL']

    def estops(self):
        return [v for k, v in self.events if k == 'ESTOP']


# Things a person might plausibly say in the room that must NOT drive the car.
CONVERSATION = [
    'i should go to the air cooler later',
    'so then we went to the sofa and sat down',
    'can you move the chair to the corner',
    'the tv is too loud',
    'we need to stop by the fridge on the way home',
    'did you see the new television',
    'go to bed early tonight',
    'my car is parked near the door',
]


def test_conversation_ignored():
    for line in CONVERSATION:
        f = Fake(bare_stop=False)     # isolate wake gating from the bare-stop rule
        f.on_utterance(line)
        moved = bool(f.goals()) or bool(f.estops())
        check(f'IGNORED: {line!r}', not moved, f'goals={f.goals()} estops={f.estops()}')


def test_conversation_with_bare_stop_enabled():
    """With bare stop on, a stop word may halt -- but must never DRIVE."""
    f = Fake(bare_stop=True)
    f.on_utterance('we need to stop by the fridge on the way home')
    check('bare "stop" in conversation halts but never drives',
          f.goals() == [] and f.estops() == [True],
          f'goals={f.goals()} estops={f.estops()}')


def test_wake_plus_command():
    f = Fake()
    f.on_utterance('hey car go to the air cooler')
    check('"hey car go to the air cooler" -> goal "air cooler"',
          f.goals() == ['air cooler'], f'{f.goals()}')


def test_wake_variants():
    cases = {
        'hey car drive to the television': 'television',
        'hey car move to the sofa': 'sofa',
        'hey car navigate to the fridge': 'fridge',
        'hey car take me to the bed': 'bed',
        'hey car go to the office chair': 'office chair',
    }
    for said, want in cases.items():
        f = Fake()
        f.on_utterance(said)
        check(f'{said!r} -> {want!r}', f.goals() == [want], f'{f.goals()}')


def test_two_step_wake():
    """"hey car" alone arms a window; the next utterance is the command."""
    f = Fake()
    f.on_utterance('hey car')
    check('wake alone arms and prompts',
          f.status == 'armed' and ('say', 'Yes?') in f.events, f'{f.events}')
    f.on_utterance('go to the air cooler')
    check('armed window accepts the follow-up command',
          f.goals() == ['air cooler'], f'{f.goals()}')


def test_arm_window_expires():
    """After the window lapses, a bare command is ignored again."""
    f = Fake()
    f.on_utterance('hey car')
    f.armed_until = time.monotonic() - 0.01      # simulate expiry
    f.on_utterance('go to the air cooler')
    check('expired arm window ignores the command', f.goals() == [], f'{f.goals()}')


def test_stop_and_cancel():
    f = Fake()
    f.on_utterance('stop')
    check('bare "stop" engages e-stop', f.estops() == [True], f'{f.estops()}')
    f2 = Fake()
    f2.on_utterance('hey car cancel')
    check('"hey car cancel" cancels the goal',
          ('cancel_srv', None) in f2.events and f2.estops() == [], f'{f2.events}')
    f3 = Fake()
    f3.on_utterance('hey car resume')
    check('"hey car resume" releases e-stop', f3.estops() == [False], f'{f3.estops()}')


def test_unk_tokens_stripped():
    """Grammar rejections arrive as [unk]; they must not become an object name."""
    f = Fake()
    f.on_utterance('hey car go to the [unk] air cooler')
    check('[unk] stripped from the target', f.goals() == ['air cooler'], f'{f.goals()}')


def test_wake_without_target_reprompts():
    """"go to the" with no object must re-prompt, not send the literal "the"."""
    f = Fake()
    f.on_utterance('hey car go to the')
    check('incomplete command re-prompts instead of guessing',
          f.goals() == [] and any(k == 'say' for k, _ in f.events), f'{f.events}')


def test_question_words_are_transcribable():
    """A word absent from the restricted grammar CANNOT be transcribed.

    This failed silently once: "in" was missing, so every "...in the room"
    question was quietly mangled before any parsing happened. Cheap to assert,
    impossible to notice by reading the code.
    """
    import os as _os
    vocab = _os.path.join(
        _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))),
        'qcar2_ws', 'src', 'qcar2_rviz_gui', 'config', 'object_vocabulary.yaml')

    class G(vc.VoiceCommand):
        def __init__(self):
            self.wake = 'hey car'

        def get_logger(self):
            class L:
                def warn(self, *_a, **_k): pass
                def info(self, *_a, **_k): pass
            return L()

    words = set(G().build_grammar(vocab))
    phrases = [
        'hey car what is around you',
        'hey car what is in the room',
        'hey car is there a cooler in the room',
        'hey car how many chairs are there',
        'hey car how many people are there',
        'hey car how far is the air cooler',
        'hey car where is the sofa',
        'hey car what can you see',
    ]
    for phrase in phrases:
        missing = [w for w in phrase.split() if w.isalpha() and w not in words]
        check(f'grammar covers {phrase!r}', not missing, f'missing {missing}')


def test_questions_route_to_the_assistant_not_the_driver():
    """"how far is the sofa" contains an object name; it must be ANSWERED,
    not obeyed as an order to drive to the sofa."""
    class V(Fake):
        def __init__(self):
            super().__init__()
            self.asked = []

        @property
        def ask_pub(self):
            return Fake._Pub(self.events, 'ASK')

        def asks(self):
            return [v for k, v in self.events if k == 'ASK']

    questions = [
        'hey car what is around you',
        'hey car how many chairs are there',
        'hey car how far is the air cooler',
        'hey car where is the sofa',
        'hey car is there a cooler in the room',
        'hey car what can you see',
    ]
    for q in questions:
        v = V()
        v.on_utterance(q)
        check(f'question routed to assistant: {q!r}',
              bool(v.asks()) and not v.goals(),
              f'asked={v.asks()} goals={v.goals()}')
    # ...while an actual order still drives.
    v = V()
    v.on_utterance('hey car go to the air cooler')
    check('a real command still drives', v.goals() == ['air cooler'] and not v.asks(),
          f'goals={v.goals()} asked={v.asks()}')


class _Msg:
    def __init__(self, data):
        self.data = data


def test_half_duplex_mutes_while_car_speaks():
    """The mic must not hear -- and act on -- the car's own voice."""
    f = Fake()
    f.muted_until, f.flush_pending = 0.0, False
    check('not muted at rest', not f.muted())
    f.on_speaking(_Msg(True))
    check('muted while the car is speaking', f.muted())
    f.on_speaking(_Msg(False))
    check('still muted for the reverb/buffer tail right after', f.muted())
    check('recogniser flush requested (discard half-heard self-speech)', f.flush_pending)
    f.muted_until = time.monotonic() - 0.01          # tail elapsed
    check('listening again once the tail has passed', not f.muted())


def test_mute_cannot_stick_forever():
    """If the announcer dies mid-sentence and never sends False, the mic
    must recover on its own instead of staying deaf."""
    f = Fake()
    f.muted_until, f.flush_pending = 0.0, False
    f.on_speaking(_Msg(True))
    remaining = f.muted_until - time.monotonic()
    check('a single mute is bounded', 0 < remaining <= vc.VoiceCommand.MAX_MUTE_SEC + 0.1,
          f'{remaining:.1f}s')


def test_prompt_does_not_eat_the_answer_window():
    """"hey car" -> "Yes?": the listening window must start AFTER "Yes?"."""
    f = Fake()
    f.muted_until, f.flush_pending = 0.0, False
    f.on_utterance('hey car')                        # arms + says "Yes?"
    f.on_speaking(_Msg(True))
    time.sleep(0.05)
    f.on_speaking(_Msg(False))
    window = f.armed_until - f.muted_until
    check('full arm window remains after the prompt finishes',
          abs(window - f.arm_seconds) < 0.05, f'{window:.2f}s of {f.arm_seconds}s')


def test_heard_card_records_decisions():
    """The "Heard" card must say what was DONE with each utterance -- that is
    what tells the user the mic works and why nothing happened."""
    f = Fake()
    f.on_utterance('the tv is too loud')
    check('conversation logged as ignored, with the reason',
          f.history[0]['text'] == 'the tv is too loud'
          and 'no "hey car"' in f.history[0]['result'], f'{f.history[0]}')
    f.on_utterance('hey car go to the air cooler')
    check('command logged with its action',
          'air cooler' in f.history[0]['result'], f'{f.history[0]}')
    f.on_utterance('hey car')
    check('bare wake phrase logged as waiting for a command',
          'say your command' in f.history[0]['result'], f'{f.history[0]}')


def test_unrecognised_speech_is_shown_and_collapsed():
    """Out-of-vocabulary speech proves the mic hears you; it must show up,
    but a run of it must not flood the card."""
    f = Fake()
    for _ in range(5):
        f.on_utterance('[unk] [unk]')
    check('unrecognised speech appears on the card',
          f.history and 'do not have words' in f.history[0]['text'], f'{list(f.history)}')
    check('a run of it collapses into one counted line',
          len(f.history) == 1 and f.history[0].get('n') == 5, f'{list(f.history)}')


TESTS = (test_heard_card_records_decisions, test_unrecognised_speech_is_shown_and_collapsed,
         test_half_duplex_mutes_while_car_speaks, test_mute_cannot_stick_forever,
         test_prompt_does_not_eat_the_answer_window,
         test_question_words_are_transcribable,
         test_questions_route_to_the_assistant_not_the_driver,
         test_conversation_ignored, test_conversation_with_bare_stop_enabled,
         test_wake_plus_command, test_wake_variants, test_two_step_wake,
         test_arm_window_expires, test_stop_and_cancel, test_unk_tokens_stripped,
         test_wake_without_target_reprompts)

if __name__ == '__main__':
    for fn in TESTS:
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
    sys.exit(1 if FAIL else 0)
