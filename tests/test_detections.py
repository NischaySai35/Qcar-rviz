#!/usr/bin/python3
"""Offline checks for the live detection boxes published by qcar2_object_mapper.

Run with scripts/run_tests.sh. Never constructs a Node / calls rclpy.init().

Covers what the console overlay depends on: coordinates normalised to the
image, the SELF-CORRECTED name shown on a box, and stable "person N" numbers.
"""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qcar2_ws', 'src', 'qcar2_rviz_gui', 'scripts'))
import qcar2_object_mapper as om  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


class Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(json.loads(msg.data))


class Stub(om.ObjectMapper):
    """Real box/person logic, fake node plumbing."""

    def __init__(self):
        self.lock = threading.Lock()
        self.landmarks = {}
        self.latest_boxes = {}
        self.person_numbers = {}
        self.transient_ttl = 20.0
        self.boxes_pub = Pub()
        self.box_tracks = {}
        self.box_seq = 0
        self.box_hold = 2.0
        self.conf_thresh = 0.25
        self.overrides = {}
        self.ignore_set = {'floor', 'wooden floor', 'wall', 'ceiling'}
        self.max_box_area = 0.40
        self.focus = 'all'
        self.cameras = ['front', 'rear', 'left', 'right']

    def add(self, lid, label, transient=False, seen_ago=0.0):
        lm = om.Landmark(lid, label, 1.0, 1.0, 1.0, 0.8, 'front',
                         time.time() - seen_ago, transient, 1.0, 6.0)
        lm.hits = 3
        self.landmarks[lid] = lm
        return lm


W, H = 820, 616


def last(stub, cam='front'):
    return stub.boxes_pub.msgs[-1]['cameras'][cam]['boxes']


def test_coordinates_are_normalised():
    s = Stub()
    s.publish_boxes('front', W, H, [(82.0, 61.6, 410.0, 308.0, 'chair', 0.9, 1.5, None)])
    b = last(s)[0]
    check('x normalised to image width', abs(b['x1'] - 0.1) < 1e-3 and abs(b['x2'] - 0.5) < 1e-3,
          f"{b['x1']}, {b['x2']}")
    check('y normalised to image height', abs(b['y1'] - 0.1) < 1e-3 and abs(b['y2'] - 0.5) < 1e-3,
          f"{b['y1']}, {b['y2']}")


def test_boxes_clamped_to_image():
    """Detector boxes can overhang the frame; the overlay must not draw off it."""
    s = Stub()
    s.publish_boxes('front', W, H, [(-20.0, -5.0, 900.0, 700.0, 'sofa', 0.7, None, None)])
    b = last(s)[0]
    check('clamped to 0..1', b['x1'] == 0.0 and b['y1'] == 0.0 and b['x2'] == 1.0 and b['y2'] == 1.0,
          f'{b}')


def test_box_shows_corrected_name():
    """If the landmark was relabelled bed -> sofa, the box says sofa even on a
    frame where the detector wobbled back to 'bed'."""
    s = Stub()
    lm = s.add(7, 'bed')
    lm.votes = {'bed': 1.0, 'sofa': 5.0}              # corrected to sofa
    s.publish_boxes('front', W, H, [(10, 10, 100, 100, 'bed', 0.5, 1.2, 7)])
    b = last(s)[0]
    check('box shows the self-corrected label', b['label'] == 'sofa', f"label={b['label']}")
    check('raw detector label kept for the tooltip', b['raw'] == 'bed', f"raw={b['raw']}")


def test_unresolved_box_still_drawn():
    """No LiDAR range yet -> still shown (amber in the UI), just without distance."""
    s = Stub()
    s.publish_boxes('front', W, H, [(10, 10, 100, 100, 'television', 0.6, None, None)])
    b = last(s)[0]
    check('unresolved detection still published', b['label'] == 'television' and b['range'] is None,
          f'{b}')


def test_people_numbered_1_2():
    s = Stub()
    s.add(11, 'person', transient=True)
    s.add(12, 'person', transient=True)
    s.publish_boxes('front', W, H, [(10, 10, 60, 200, 'person', 0.8, 2.0, 11),
                                    (300, 10, 360, 200, 'person', 0.8, 2.5, 12)])
    labels = sorted(b['label'] for b in last(s))
    check('two people are "person 1" and "person 2"', labels == ['person 1', 'person 2'],
          f'{labels}')
    check('people are marked as kind=person', all(b['kind'] == 'person' for b in last(s)))


def test_person_number_is_stable():
    """The same person must keep their number frame to frame."""
    s = Stub()
    s.add(21, 'person', transient=True)
    s.add(22, 'person', transient=True)
    first = {lid: s.person_number(lid) for lid in (21, 22)}
    again = {lid: s.person_number(lid) for lid in (22, 21)}      # order swapped
    check('numbers stable across frames regardless of order', first == again,
          f'{first} vs {again}')


def test_person_number_is_reused_after_they_leave():
    """Person 1 walks out -> the next newcomer becomes person 1, not person 3."""
    s = Stub()
    s.add(31, 'person', transient=True)
    s.add(32, 'person', transient=True)
    s.person_number(31)
    s.person_number(32)
    s.landmarks[31].last_seen = time.time() - 999               # person 1 has left
    s.add(33, 'person', transient=True)
    n = s.person_number(33)
    check('freed number is reused', n == 1, f'newcomer got person {n}')
    check('person still present keeps theirs', s.person_number(32) == 2)


def test_per_camera_state_kept():
    """360 view: each camera's boxes are kept and published together."""
    s = Stub()
    s.publish_boxes('front', W, H, [(10, 10, 100, 100, 'chair', 0.9, 1.0, None)])
    s.publish_boxes('left', W, H, [(10, 10, 100, 100, 'desk', 0.9, 1.0, None)])
    cams = s.boxes_pub.msgs[-1]['cameras']
    check('both cameras present in one message', set(cams) == {'front', 'left'}, f'{set(cams)}')
    check('front boxes survive a later left update', cams['front']['boxes'][0]['label'] == 'chair')


def test_focus_schedule():
    """Single front view -> front gets every other detection slot."""
    s = Stub()
    s.focus = 'all'
    check('360 view: plain round-robin', s.schedule() == ['front', 'rear', 'left', 'right'],
          f'{s.schedule()}')
    s.focus = 'front'
    sched = s.schedule()
    check('front view: front gets half of all slots',
          sched.count('front') == len(sched) / 2, f'{sched}')
    check('front view: other cameras still get slots',
          {'rear', 'left', 'right'} <= set(sched), f'{sched}')


# --------------------------------------------------------- floor-as-desk

def test_floor_shaped_box_rejected():
    """The screenshot bug: the whole floor boxed as "desk"."""
    s = Stub()
    # Full width, bottom half of the frame -- exactly what was on screen.
    check('full-width box along the bottom edge is rejected',
          not s.plausible('desk', 0, 0.5 * H, W, H, W, H))


def test_huge_box_rejected():
    s = Stub()
    check('box covering >40% of the frame is rejected',
          not s.plausible('sofa', 0, 0, 0.8 * W, 0.6 * H, W, H))


def test_background_labels_rejected():
    s = Stub()
    for lab in ('floor', 'wall', 'ceiling', 'Wooden Floor'):
        check(f'background label "{lab}" is never shown or mapped',
              not s.plausible(lab, 100, 100, 200, 200, W, H))


def test_real_furniture_accepted():
    s = Stub()
    check('an ordinary air-cooler-sized box is accepted',
          s.plausible('air cooler', 0.45 * W, 0.3 * H, 0.52 * W, 0.62 * H, W, H))
    check('a wide-but-short sofa off the bottom edge is accepted',
          s.plausible('sofa', 0.05 * W, 0.35 * H, 0.9 * W, 0.7 * H, W, H))


def test_max_area_override():
    """A big item seen up close can be allowed a bigger box, per class."""
    s = Stub()
    s.overrides = {'bed': {'max_area': 0.7}}
    check('per-class max_area override is honoured',
          s.plausible('bed', 0, 0, 0.9 * W, 0.6 * H, W, H))


# ---------------------------------------------------------------- flicker

def box(conf, x=100.0, lid=None, raw='air cooler', rng=None):
    return (x, 100.0, x + 60.0, 220.0, raw, conf, rng, lid)


def test_box_survives_a_missed_frame():
    """The reported flicker: one frame without the object must not blank it."""
    s = Stub()
    s.publish_boxes('front', W, H, [box(0.40)])
    first = last(s)
    s.publish_boxes('front', W, H, [])                   # detector missed it
    held = last(s)
    check('box still shown after one missed frame', len(held) == 1, f'{held}')
    check('held box is marked stale (drawn faded)', held and held[0]['stale'] is True)
    check('same on-screen identity (no re-create blink)',
          held and held[0]['key'] == first[0]['key'])


def test_box_expires_after_hold():
    s = Stub()
    s.publish_boxes('front', W, H, [box(0.40)])
    for t in s.box_tracks['front']:
        t['last'] -= 5.0                                 # long gone
    s.publish_boxes('front', W, H, [])
    check('box removed once the hold time has passed', last(s) == [], f'{last(s)}')


def test_weak_detection_keeps_but_cannot_create():
    """Hysteresis: a weak frame sustains a shown box but cannot start one."""
    s = Stub()
    s.publish_boxes('front', W, H, [box(0.15, x=500.0)])  # weak, new
    check('a weak detection alone does not create a box', last(s) == [], f'{last(s)}')
    s.publish_boxes('front', W, H, [box(0.40)])           # strong -> shown
    s.publish_boxes('front', W, H, [box(0.15, x=103.0)])  # weak, same place
    b = last(s)
    check('a weak re-detection keeps the box live (not stale)',
          len(b) == 1 and b[0]['stale'] is False, f'{b}')


def test_weak_frame_keeps_person_number():
    """A person's box must not flip between "person 2" and bare "person"."""
    s = Stub()
    s.add(41, 'person', transient=True)
    s.publish_boxes('front', W, H, [box(0.6, lid=41, raw='person', rng=2.0)])
    named = last(s)[0]['label']
    s.publish_boxes('front', W, H, [box(0.15, x=104.0, raw='person')])   # weak, unresolved
    kept = last(s)[0]
    check('weak frame keeps the person number', kept['label'] == named,
          f'{named!r} -> {kept["label"]!r}')
    check('and keeps the last known distance', kept['range'] == 2.0, f'{kept["range"]}')


def test_moving_box_keeps_identity():
    """A walking person's box moves between frames; it must be the same box."""
    s = Stub()
    s.publish_boxes('front', W, H, [box(0.5, x=100.0, raw='person')])
    k1 = last(s)[0]['key']
    s.publish_boxes('front', W, H, [box(0.5, x=130.0, raw='person')])
    b = last(s)
    check('moved box matched to the same track', len(b) == 1 and b[0]['key'] == k1, f'{b}')


TESTS = (test_floor_shaped_box_rejected, test_huge_box_rejected,
         test_background_labels_rejected, test_real_furniture_accepted,
         test_max_area_override, test_box_survives_a_missed_frame,
         test_box_expires_after_hold, test_weak_detection_keeps_but_cannot_create,
         test_weak_frame_keeps_person_number, test_moving_box_keeps_identity,
         test_coordinates_are_normalised, test_boxes_clamped_to_image,
         test_box_shows_corrected_name, test_unresolved_box_still_drawn,
         test_people_numbered_1_2, test_person_number_is_stable,
         test_person_number_is_reused_after_they_leave,
         test_per_camera_state_kept, test_focus_schedule)

if __name__ == '__main__':
    for fn in TESTS:
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
    sys.exit(1 if FAIL else 0)
