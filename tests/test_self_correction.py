#!/usr/bin/python3
"""Offline checks for landmark self-correction in qcar2_object_mapper.

Run with scripts/run_tests.sh. Never constructs a Node / calls rclpy.init().

Covers the three ways a landmark must be able to correct itself:
  * WHAT  -- a wrong label is outvoted by later, closer sightings
  * WHERE -- a position built from distant sightings is still pullable by a
             close pass (this is what the weight cap buys)
  * WHETHER -- a landmark repeatedly looked for and not found is deleted
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qcar2_ws', 'src', 'qcar2_rviz_gui', 'scripts'))
import qcar2_object_mapper as om  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


MAXW = 6.0          # node default: max_weight


def weight(conf, rng):
    """Same formula the node uses."""
    return conf / (0.3 + 0.7 * max(0.0, rng))


def new(label, x, y, conf, rng):
    return om.Landmark(1, label, x, y, weight(conf, rng), conf, 'front',
                       0.0, False, rng, MAXW)


def observe(lm, label, x, y, conf, rng, t=1.0, cam='front'):
    lm.update(label, x, y, weight(conf, rng), conf, cam, t, rng)


# ----------------------------------------------------------------- WHAT

def test_label_starts_as_first_guess():
    lm = new('bed', 3.0, 1.0, 0.4, 4.5)
    check('label starts as the first guess', lm.label == 'bed', lm.label)


def test_close_sightings_outvote_a_wrong_label():
    """Seen once from 4.5 m as a bed; then four times from ~1 m as a sofa."""
    lm = new('bed', 3.0, 1.0, 0.45, 4.5)
    for _ in range(4):
        observe(lm, 'sofa', 3.0, 1.0, 0.72, 1.0)
    check('wrong distant label is corrected by close sightings',
          lm.label == 'sofa', f'label={lm.label} votes={ {k: round(v,2) for k,v in lm.votes.items()} }')


def test_single_close_sighting_does_not_flip_a_well_established_label():
    """Self-correction must not mean 'believes the most recent frame'."""
    lm = new('sofa', 3.0, 1.0, 0.7, 1.2)
    for _ in range(9):
        observe(lm, 'sofa', 3.0, 1.0, 0.7, 1.2)
    observe(lm, 'bed', 3.0, 1.0, 0.55, 1.2)      # one dissenting frame
    check('one odd frame does not flip an established label',
          lm.label == 'sofa', f'label={lm.label}')


def test_label_confidence_reported():
    lm = new('sofa', 3.0, 1.0, 0.7, 1.2)
    for _ in range(4):
        observe(lm, 'sofa', 3.0, 1.0, 0.7, 1.2)
    clean = lm.label_confidence
    observe(lm, 'bed', 3.0, 1.0, 0.7, 1.2)
    check('label_confidence is 1.0 when every sighting agreed',
          abs(clean - 1.0) < 1e-9, f'{clean:.3f}')
    check('label_confidence drops once sightings disagree',
          lm.label_confidence < 1.0, f'{lm.label_confidence:.3f}')
    d = lm.as_dict()
    check('alternatives are reported for transparency',
          any(a['label'] == 'bed' for a in d['alternatives']), f"{d['alternatives']}")


# ---------------------------------------------------------------- WHERE

def test_close_pass_corrects_position():
    """20 distant sightings put it at x=3.0; the object is really at x=3.6.

    Without the weight cap the accumulated history would pin it near 3.0.
    """
    lm = new('air cooler', 3.0, 0.0, 0.5, 5.0)
    for _ in range(19):
        observe(lm, 'air cooler', 3.0, 0.0, 0.5, 5.0)
    drifted = lm.x
    # ~12 sightings is a second or two of driving past it at the round-robin
    # detection rate -- i.e. one ordinary close pass, not a contrived number.
    for _ in range(12):
        observe(lm, 'air cooler', 3.6, 0.0, 0.8, 0.8)   # close, confident
    check('position was stuck near the distant estimate before correction',
          abs(drifted - 3.0) < 0.05, f'x={drifted:.3f}')
    check('close-range sightings pull the position to the truth',
          abs(lm.x - 3.6) < 0.15, f'x={lm.x:.3f} (want ~3.60)')


def test_weight_is_capped():
    lm = new('chair', 1.0, 1.0, 0.9, 0.5)
    for _ in range(200):
        observe(lm, 'chair', 1.0, 1.0, 0.9, 0.5)
    check('accumulated weight is capped so it stays correctable',
          lm.weight <= MAXW + 1e-9, f'weight={lm.weight:.2f} cap={MAXW}')


def test_position_still_averages_noise():
    """Correctable must not mean jumpy: noise should still average out."""
    lm = new('desk', 2.0, 0.0, 0.6, 2.0)
    for i in range(40):
        jitter = 0.10 if i % 2 else -0.10
        observe(lm, 'desk', 2.0 + jitter, 0.0, 0.6, 2.0)
    check('symmetric noise still averages out', abs(lm.x - 2.0) < 0.05, f'x={lm.x:.3f}')


def test_best_range_tracked():
    lm = new('tv', 3.0, 0.0, 0.5, 4.0)
    observe(lm, 'tv', 3.0, 0.0, 0.7, 0.9)
    check('closest approach recorded', abs(lm.best_range - 0.9) < 1e-9, f'{lm.best_range}')


# -------------------------------------------------------------- WHETHER

def test_misses_accumulate_and_hits_forgive():
    lm = new('vase', 2.0, 0.0, 0.5, 2.0)
    for _ in range(3):
        lm.miss()
    check('misses accumulate', lm.misses == 3.0, f'{lm.misses}')
    observe(lm, 'vase', 2.0, 0.0, 0.5, 2.0)
    check('seeing it again forgives a miss', lm.misses == 2.0, f'{lm.misses}')


def test_prune_condition():
    """Mirrors prune_pass(): misses >= max_misses AND misses > hits."""
    max_misses = 6.0

    def prunable(lm):
        return lm.misses >= max_misses and lm.misses > lm.hits

    ghost = new('bottle', 5.0, 5.0, 0.3, 5.5)      # 1 hit
    for _ in range(7):
        ghost.miss()
    check('a one-off false positive is pruned', prunable(ghost),
          f'hits={ghost.hits} misses={ghost.misses}')

    real = new('sofa', 3.0, 1.0, 0.8, 1.0)
    for _ in range(20):
        observe(real, 'sofa', 3.0, 1.0, 0.8, 1.0)
    for _ in range(6):
        real.miss()
    check('a well-established object is NOT pruned by a few misses',
          not prunable(real), f'hits={real.hits} misses={real.misses}')


def test_absorb_merges_votes():
    a = new('sofa', 3.0, 1.0, 0.7, 1.0)
    b = om.Landmark(2, 'bed', 3.1, 1.0, weight(0.6, 1.0), 0.6, 'left',
                    1.0, False, 1.0, MAXW)
    a.absorb(b)
    check('merging keeps both labels as evidence',
          set(a.votes) == {'sofa', 'bed'}, f'{set(a.votes)}')
    check('merged position lies between the two', 3.0 <= a.x <= 3.1, f'x={a.x:.3f}')
    check('merged hits are summed', a.hits == 2, f'{a.hits}')


TESTS = (test_label_starts_as_first_guess, test_close_sightings_outvote_a_wrong_label,
         test_single_close_sighting_does_not_flip_a_well_established_label,
         test_label_confidence_reported, test_close_pass_corrects_position,
         test_weight_is_capped, test_position_still_averages_noise,
         test_best_range_tracked, test_misses_accumulate_and_hits_forgive,
         test_prune_condition, test_absorb_merges_votes)

if __name__ == '__main__':
    for fn in TESTS:
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
    sys.exit(1 if FAIL else 0)
