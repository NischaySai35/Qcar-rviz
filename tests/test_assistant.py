#!/usr/bin/python3
"""Offline checks for qcar2_assistant question parsing and answering.

Run with scripts/run_tests.sh. Never constructs a Node / calls rclpy.init().

The point of these: the assistant must answer from the MAP, and must be
honest when it does not know. Numbers are counted, distances are measured --
never phrased from a language model.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qcar2_ws', 'src', 'qcar2_rviz_gui', 'scripts'))
import qcar2_assistant as qa  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


SYNONYMS = {'cooler': 'air cooler', 'ac': 'air cooler', 'air cooler': 'air cooler',
            'tv': 'television', 'television': 'television', 'couch': 'sofa',
            'sofa': 'sofa', 'fridge': 'refrigerator', 'refrigerator': 'refrigerator',
            'chair': 'chair', 'desk': 'desk', 'bed': 'bed'}

OBJECTS = [
    {'label': 'air cooler', 'x': 4.0, 'y': 0.0, 'label_confidence': 0.95, 'alternatives': []},
    {'label': 'chair', 'x': 1.0, 'y': 1.0, 'label_confidence': 0.9, 'alternatives': []},
    {'label': 'chair', 'x': 1.0, 'y': -1.0, 'label_confidence': 0.9, 'alternatives': []},
    {'label': 'chair', 'x': 2.0, 'y': 2.0, 'label_confidence': 0.9, 'alternatives': []},
    {'label': 'sofa', 'x': -3.0, 'y': 0.0, 'label_confidence': 0.52,
     'alternatives': [{'label': 'bed', 'score': 3.1}]},
    {'label': 'desk', 'x': 0.0, 'y': 3.0, 'label_confidence': 0.88, 'alternatives': []},
]


class Fake(qa.Assistant):
    """Real logic, fake I/O."""

    def __init__(self, objects=None, transient=None, pose=(0.0, 0.0, 0.0), age=0.0):
        self.synonyms = dict(SYNONYMS)
        self.vocab = sorted({v for v in SYNONYMS.values()})
        self.objects = list(OBJECTS if objects is None else objects)
        self.transient = list(transient or [])
        self.stats = {}
        self.use_llm = False
        self.llm_ok = False
        self.stale_after = 25.0
        self.live_check_sec = 1.0
        self._pose = pose
        self._age = age
        self.said = []
        self.navs = []
        self.explores = []
        import threading
        self.lock = threading.Lock()

    @property
    def last_objects_at(self):
        import time as _t
        return _t.monotonic() - self._age

    @last_objects_at.setter
    def last_objects_at(self, v):
        pass

    def get_logger(self):
        class L:
            def info(self, *_a, **_k): pass
            def warn(self, *_a, **_k): pass
            def error(self, *_a, **_k): pass
        return L()

    def car_pose(self):
        return self._pose

    def say(self, text):
        self.said.append(text)

    class _Pub:
        def __init__(self, sink):
            self.sink = sink

        def publish(self, msg):
            self.sink.append(getattr(msg, 'data', None))

    @property
    def nav_pub(self):
        return Fake._Pub(self.navs)

    @property
    def explore_pub(self):
        return Fake._Pub(self.explores)


# ------------------------------------------------------------------ parsing

def test_parse_intents():
    a = Fake()
    cases = {
        'is there a cooler in the room': 'exists',
        'is there an air cooler here': 'exists',
        'how many chairs are there': 'count',
        'how many people are there': 'count',
        'how far is the air cooler': 'distance',
        'what is the distance to the sofa': 'distance',
        'where is the desk': 'where',
        'what can you see': 'list',
        'what objects are in the room': 'list',
        'go to the air cooler': 'navigate',
    }
    for q, want in cases.items():
        got = a.parse(q).get('intent')
        check(f'parse({q!r}) -> {want}', got == want, f'got {got!r}')


def test_resolve_synonyms_and_plurals():
    a = Fake()
    check('resolve "cooler"', a.resolve('cooler') == 'air cooler')
    check('resolve "chairs" (plural)', a.resolve('chairs') == 'chair', a.resolve('chairs'))
    check('resolve "people" -> person', a.resolve('people') == 'person')
    check('resolve "anyone" -> person', a.resolve('anyone') == 'person')
    check('unknown noun resolves to None', a.resolve('helicopter') is None)


# ----------------------------------------------------------------- answering

def test_exists_yes_and_no():
    a = Fake()
    text, data = a.answer('is there a cooler in the room')
    check('existing object -> yes', data.get('exists') is True and text.lower().startswith('yes'),
          text)
    text, data = a.answer('is there a refrigerator in the room')
    check('absent object -> no', data.get('exists') is False and text.lower().startswith('no'),
          text)


def test_count_is_counted_not_guessed():
    a = Fake()
    text, data = a.answer('how many chairs are there')
    check('counts the actual landmarks', data.get('count') == 3, f'{data} / {text}')
    check('count is phrased in words', 'three' in text.lower(), text)
    _t, data = a.answer('how many beds are there')
    check('zero count is reported honestly', data.get('count') == 0, f'{data}')


def test_distance_is_measured_from_the_car():
    a = Fake(pose=(0.0, 0.0, 0.0))
    text, data = a.answer('how far is the air cooler')
    check('distance measured from current pose',
          abs(data['distance_m'] - 4.0) < 0.01, f'{data.get("distance_m")}')
    check('distance appears in the sentence', '4.0' in text, text)
    # Move the car; the same question must give a different answer.
    b = Fake(pose=(3.0, 0.0, 0.0))
    _t, d2 = b.answer('how far is the air cooler')
    check('answer tracks the car moving', abs(d2['distance_m'] - 1.0) < 0.01,
          f'{d2.get("distance_m")}')


def test_where_gives_direction():
    a = Fake(pose=(0.0, 0.0, 0.0))
    text, data = a.answer('where is the air cooler')
    check('object dead ahead is described as ahead', 'straight ahead' in text, text)
    # Facing +Y: the cooler at +X is now on the right.
    b = Fake(pose=(0.0, 0.0, math.radians(90)))
    text2, _d = b.answer('where is the air cooler')
    check('direction is relative to car heading', 'right' in text2, text2)


def test_nearest_of_several_is_used():
    a = Fake(pose=(0.0, 0.0, 0.0))
    _t, data = a.answer('where is the chair')
    check('nearest matching object chosen',
          abs(data['distance_m'] - math.hypot(1.0, 1.0)) < 0.01, f'{data.get("distance_m")}')
    check('but the total count is still reported', data.get('count') == 3, f'{data}')


def test_uncertain_label_is_hedged():
    """The map can be sure something is there yet unsure what it is."""
    a = Fake(pose=(0.0, 0.0, 0.0))
    text, _d = a.answer('where is the sofa')
    check('low label confidence is admitted, not hidden',
          'not certain' in text.lower() and 'bed' in text.lower(), text)


def test_unknown_object_says_so():
    a = Fake()
    text, data = a.answer('is there a helicopter in the room')
    check('unknown noun -> honest "I do not know"',
          'do not know' in text.lower() and 'unresolved' in data, text)
    check('and it lists what it does know', 'air cooler' in text, text)


def test_list_summarises_room():
    a = Fake()
    text, data = a.answer('what can you see')
    check('list counts each kind', data['counts'].get('chair') == 3, f"{data['counts']}")
    check('list mentions the cooler', 'air cooler' in text, text)


def test_no_pose_is_admitted():
    a = Fake(pose=None)
    text, data = a.answer('how far is the air cooler')
    check('cannot measure without localisation, and says so',
          data.get('no_pose') is True and 'where i am' in text.lower(), text)


# -------------------------------------------------------------------- people

def test_people_answered_live():
    a = Fake(transient=[{'label': 'person', 'x': 2.0, 'y': 0.0}], age=1.0)
    text, data = a.answer('how many people are there')
    check('people counted from live sightings', data.get('count') == 1, f'{data}')
    check('phrased as right now', 'right now' in text.lower(), text)


def test_people_stale_is_flagged():
    a = Fake(transient=[{'label': 'person', 'x': 2.0, 'y': 0.0}], age=120.0)
    text, data = a.answer('how many people are there')
    check('stale person data is flagged, not passed off as current',
          data.get('stale') is True, f'{data}')
    check('and offers to go and check', 'go and check' in text.lower(), text)


def test_people_none_is_hedged():
    a = Fake(transient=[], age=1.0)
    text, data = a.answer('is there anyone here')
    check('no one seen -> count 0', data.get('count') == 0, f'{data}')
    check('admits it only sees where cameras point',
          'cameras' in text.lower(), text)


def test_go_and_check_triggers_a_sweep():
    a = Fake(transient=[], age=1.0)
    text, data = a.answer('how many people are there go and check')
    check('explicit "go and check" starts a live sweep',
          data.get('live_check') is True and a.explores == [True], f'{data} {a.explores}')
    check('and says it is going to look', 'look' in text.lower(), text)


def test_navigate_intent_delegates():
    a = Fake()
    _t, data = a.answer('go to the air cooler')
    check('navigate question delegates to object nav',
          a.navs and 'cooler' in a.navs[0], f'{a.navs}')
    check('and reports the resolved label', data.get('target') == 'air cooler', f'{data}')


TESTS = (test_parse_intents, test_resolve_synonyms_and_plurals,
         test_exists_yes_and_no, test_count_is_counted_not_guessed,
         test_distance_is_measured_from_the_car, test_where_gives_direction,
         test_nearest_of_several_is_used, test_uncertain_label_is_hedged,
         test_unknown_object_says_so, test_list_summarises_room,
         test_no_pose_is_admitted, test_people_answered_live,
         test_people_stale_is_flagged, test_people_none_is_hedged,
         test_go_and_check_triggers_a_sweep, test_navigate_intent_delegates)

if __name__ == '__main__':
    for fn in TESTS:
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
    sys.exit(1 if FAIL else 0)
