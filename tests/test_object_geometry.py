#!/usr/bin/python3
"""Offline geometry checks for qcar2_object_mapper.

Run with scripts/run_tests.sh (which sources the ROS environment first).

These import the node module but NEVER construct a Node and never call
rclpy.init(), so running them does not join the live ROS graph and cannot
disturb a running mapping/navigation stack.

The headline test is test_same_object_two_cameras: the "TV seen by the front
camera, then by the left camera after the car turns" case. It forward-projects
a known world object into two different cameras at two different car poses and
checks that both recover the SAME map coordinate -- which is what makes the
landmark merge instead of being mapped twice.
"""
import math
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qcar2_ws', 'src', 'qcar2_rviz_gui', 'scripts'))
import qcar2_object_mapper as om  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def rpy_to_matrix(r, p, y):
    """URDF fixed-axis RPY -> Rz(y) @ Ry(p) @ Rx(r)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def matrix_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
            y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
            y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
            y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return x, y, z, w


# Camera mounts exactly as qcar2/urdf/QCar2.urdf declares them.
URDF_CAMS = {
    'front': ((0.198, 0.0, 0.1095), (-1.5708, 0.0, -1.5708)),
    'rear':  ((-0.165, -0.0005, 0.1095), (-1.5708, 0.0, 1.5708)),
    'right': ((0.0155, -0.072, 0.1095), (-1.5708, 0.0, 3.14159)),
    'left':  ((0.0155, 0.051, 0.1095), (-1.5708, 0.0, 0.0)),
}
LIDAR_XY = np.array([-0.0139, 0.0])       # lidar_joint origin in base_link

W, H = 820, 616
HFOV = math.radians(120.0)
FX = (W / 2.0) / math.tan(HFOV / 2.0)
CX, CY = W / 2.0, H / 2.0


def test_quat_matrix_roundtrip():
    for name, (_, rpy) in URDF_CAMS.items():
        R = rpy_to_matrix(*rpy)
        back = om.quat_to_matrix(*matrix_to_quat(R))
        check(f'quat_to_matrix roundtrip [{name}]', np.allclose(R, back, atol=1e-6),
              f'max err {np.abs(R - back).max():.2e}')


def test_optical_axes():
    """Each URDF camera frame must be a real optical frame pointing outward."""
    expect = {'front': [1, 0, 0], 'rear': [-1, 0, 0], 'left': [0, 1, 0], 'right': [0, -1, 0]}
    for name, (_, rpy) in URDF_CAMS.items():
        R = rpy_to_matrix(*rpy)
        fwd, down = R @ np.array([0, 0, 1.0]), R @ np.array([0, 1.0, 0])
        check(f'optical z_opt points outward [{name}]',
              np.allclose(fwd, expect[name], atol=1e-3), f'z_opt={np.round(fwd, 3)}')
        check(f'optical y_opt points down [{name}]',
              np.allclose(down, [0, 0, -1], atol=1e-3), f'y_opt={np.round(down, 3)}')


def test_ang_norm():
    check('ang_norm wraps +3pi', abs(abs(om.ang_norm(3 * math.pi)) - math.pi) < 1e-9)
    check('ang_norm identity', abs(om.ang_norm(0.5) - 0.5) < 1e-12)


def test_image_to_numpy_padded():
    """A padded `step` must not shear the image."""
    msg = types.SimpleNamespace(encoding='bgr8', height=4, width=5, step=5 * 3 + 7,
                                data=bytes(range(4 * (5 * 3 + 7))))
    img = om.image_to_numpy(msg)
    check('image_to_numpy honours row padding',
          img is not None and img.shape == (4, 5, 3),
          f'shape={None if img is None else img.shape}')
    rgb = types.SimpleNamespace(encoding='rgb8', height=1, width=1, step=3,
                                data=bytes([10, 20, 30]))
    out = om.image_to_numpy(rgb)
    check('image_to_numpy swaps rgb8 -> bgr8',
          out is not None and list(out[0, 0]) == [30, 20, 10],
          f'{None if out is None else list(out[0, 0])}')


class Stub:
    """Minimal stand-in for the mapper's geometry-relevant attributes."""
    min_range, max_range = 0.25, 6.0
    cluster_gap, min_cluster_points = 0.35, 2
    range_in_sector = om.ObjectMapper.range_in_sector


def test_nearest_cluster_wins():
    """An object in front of a wall must return the OBJECT, not the wall."""
    obj = [(2.0, y) for y in np.linspace(-0.3, 0.3, 9)]
    wall = [(5.0, y) for y in np.linspace(-2.0, 2.0, 60)]
    pts = np.array(obj + wall, dtype=float)
    hit = Stub().range_in_sector(pts, np.array([0.0, 0.0]),
                                 math.atan2(-0.35, 2.0), math.atan2(0.35, 2.0))
    check('nearest cluster chosen over the wall behind it',
          hit is not None and abs(hit[2] - 2.0) < 0.15,
          f'range={None if hit is None else round(hit[2], 3)}')


def test_sector_wraparound():
    """The rear camera's sector straddles +-pi; it must still match."""
    pts = np.array([(-2.0, y) for y in np.linspace(-0.3, 0.3, 9)], dtype=float)
    hit = Stub().range_in_sector(pts, np.array([0.0, 0.0]),
                                 math.atan2(0.35, -2.0), math.atan2(-0.35, -2.0))
    check('sector spanning +-pi still matches',
          hit is not None and abs(hit[2] - 2.0) < 0.15,
          f'range={None if hit is None else round(hit[2], 3)}')


def test_out_of_plane_returns_none():
    """No LiDAR return in the sector -> unresolved, never an invented range.

    This is the documented 2D-LiDAR-plane limitation: a wall-mounted TV gives
    a bearing but no range, and must be skipped rather than guessed at.
    """
    pts = np.array([(0.0, 3.0), (0.1, 3.0)], dtype=float)
    hit = Stub().range_in_sector(pts, np.array([0.0, 0.0]), -0.1, 0.1)
    check('no return in sector -> None (no invented range)', hit is None)


def project_and_recover(car_xy, car_yaw, cam, world_xy, extent=0.35):
    """Forward-project a world object into `cam`, then run the node's own
    recovery path and return the map coordinate it reconstructs."""
    t_cam, rpy = URDF_CAMS[cam]
    R_opt = rpy_to_matrix(*rpy)
    o = np.array(t_cam[:2]) - LIDAR_XY
    c, s = math.cos(car_yaw), math.sin(car_yaw)
    R_map_base = np.array([[c, -s], [s, c]])
    rel_base = R_map_base.T @ (np.array(world_xy) - np.array(car_xy))
    obj_scan = rel_base - LIDAR_XY

    along = np.array([-(obj_scan - o)[1], (obj_scan - o)[0]])
    along /= np.linalg.norm(along)
    surf = [obj_scan + along * d for d in np.linspace(-extent / 2, extent / 2, 11)]
    far = obj_scan + (obj_scan - o) / np.linalg.norm(obj_scan - o) * 2.0
    wall = [far + along * d for d in np.linspace(-2.0, 2.0, 80)]
    pts = np.array([*surf, *wall])

    us = []
    for p in (surf[0], surf[-1]):
        d_opt = R_opt.T @ np.array([p[0] - o[0], p[1] - o[1], 0.0])
        if d_opt[2] <= 1e-6:
            return None                      # behind this camera
        u = FX * (d_opt[0] / d_opt[2]) + CX
        if not (0 <= u <= W):
            return None                      # outside this camera's image
        us.append(u)
    x1, x2 = min(us), max(us)

    # ---- from here on, exactly what the node does ----
    bearings = []
    shrink = (x2 - x1) * 0.15 / 2.0
    for u in (x1 + shrink, x2 - shrink):
        d = R_opt @ np.array([(u - CX) / FX, 0.0, 1.0])
        bearings.append(math.atan2(d[1], d[0]))
    hit = Stub().range_in_sector(pts, o, bearings[0], bearings[1])
    if hit is None:
        return None
    p_base = np.array([hit[0], hit[1]]) + LIDAR_XY
    return R_map_base @ p_base + np.array(car_xy)


def test_same_object_two_cameras():
    """THE test: one TV, seen by the front camera from one pose and the left
    camera from another, must land on the same map coordinate."""
    tv = (3.0, 1.0)
    a = project_and_recover((0.0, 1.0), 0.0, 'front', tv)
    # Car facing -Y has its left side toward +X, so this pose puts the same TV
    # in the LEFT camera after the car has turned.
    b = project_and_recover((0.6, 1.0), math.radians(-90), 'left', tv)
    check('front camera recovers the TV',
          a is not None and np.linalg.norm(a - np.array(tv)) < 0.30,
          f'got {None if a is None else np.round(a, 3)} want {tv}')
    check('left camera recovers the TV',
          b is not None and np.linalg.norm(b - np.array(tv)) < 0.30,
          f'got {None if b is None else np.round(b, 3)} want {tv}')
    if a is not None and b is not None:
        sep = float(np.linalg.norm(a - b))
        check('SAME OBJECT: front vs left agree within the 0.6 m association gate',
              sep < 0.60, f'separation {sep:.3f} m -> merges into ONE landmark')


def test_two_distinct_objects_stay_separate():
    """The converse: genuinely different objects must NOT be merged."""
    a = project_and_recover((0.0, 0.0), 0.0, 'front', (3.0, -0.9))
    b = project_and_recover((0.0, 0.0), 0.0, 'front', (3.0, 0.9))
    if a is None or b is None:
        check('two distinct objects both recovered', False, f'a={a} b={b}')
        return
    sep = float(np.linalg.norm(a - b))
    check('DISTINCT OBJECTS stay separate (> 0.6 m gate)', sep > 0.60,
          f'separation {sep:.3f} m -> two landmarks')


TESTS = (test_quat_matrix_roundtrip, test_optical_axes, test_ang_norm,
         test_image_to_numpy_padded, test_nearest_cluster_wins,
         test_sector_wraparound, test_out_of_plane_returns_none,
         test_same_object_two_cameras, test_two_distinct_objects_stay_separate)

if __name__ == '__main__':
    for fn in TESTS:
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
    sys.exit(1 if FAIL else 0)
