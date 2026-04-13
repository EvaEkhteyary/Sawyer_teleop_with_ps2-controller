#!/usr/bin/env python3
"""
ps2_sawyer_teleop_viz.py — Generic USB gamepad teleop for Sawyer + RViz
========================================================================
Confirmed button/axis indices from your controller:
  A=0  B=1  X=2  Y=3  L=4  R=5  Select=6  Start=7  Mode=8
  Left stick click=9  Right stick click=10

CONTROL SCHEME
══════════════
  POSITION (hold L to enable all motion)
    Left  stick  fwd/back   → EE +X / -X
    Left  stick  left/right → EE +Y / -Y
    Right stick  fwd/back   → EE +Z / -Z  (up/down)

  ORIENTATION  (two modes, toggle with Y button)
    Mode 0 — YAW only (default, easiest for pick-and-place):
      Right stick  left/right → yaw

    Mode 1 — ROLL + PITCH (for fine orientation control):
      Right stick  left/right → roll
      Right stick  fwd/back   → pitch
      (yaw is frozen in this mode)

  GRIPPER
    R  (button 5)   → toggle open / close

  HOMING
    A  (button 0)   → startup: move to HOME (safe interpolation)
    B  (button 1)   → startup: skip home, start from current position
    Start (button 7)→ mid-session: return to HOME safely
    Select(button 6)→ mid-session: re-anchor IK at current position

  ORIENTATION HELPERS
    X  (button 2)   → reset orientation to home (clears accumulated drift)
    Y  (button 3)   → toggle orientation mode  (yaw-only ↔ roll+pitch)

  KEYBOARD
    r               → same as Start (return to HOME)
"""

import sys
import os
import math
import importlib.util
import intera_interface

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
import pygame

# ─── RelaxedIK ─────────────────────────────────────────────────────────────
_RIK_ROOT     = '/root/catkin_ws/src/relaxed_ik_core'
_RIK_WRAPPER  = _RIK_ROOT + '/wrappers/python_wrapper.py'
_RIK_SETTINGS = _RIK_ROOT + '/configs/settings.yaml'

_spec = importlib.util.spec_from_file_location("python_wrapper", _RIK_WRAPPER)
_mod  = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
RelaxedIKRust = _mod.RelaxedIKRust

# ─── Robot constants ───────────────────────────────────────────────────────
JOINT_NAMES = [
    'right_j0', 'right_j1', 'right_j2',
    'right_j3', 'right_j4', 'right_j5', 'right_j6'
]
GRIPPER_JOINT_NAMES = [
    'finger_joint',
    'right_outer_knuckle_joint',
    'left_inner_knuckle_joint',  'right_inner_knuckle_joint',
    'left_inner_finger_joint',   'right_inner_finger_joint',
]
GRIPPER_OPEN        = 0.0
GRIPPER_CLOSE       = 0.8
DEFAULT_HOME_CONFIG = [0.3474, -1.3143, -0.5663, 1.3630, 0.0967, 1.4469, 3.0276]

# ─── Safe-homing settings ──────────────────────────────────────────────────
SAFE_HOME_SPEED       = 0.15   # 0–1 fraction of max joint speed
SAFE_HOME_TIMEOUT     = 15.0   # seconds total
SAFE_HOME_STEPS       = 8      # waypoints between current and home
SAFE_HOME_STEP_THRESH = 0.01   # rad — skip waypoint if delta < this
J6_MAX_DELTA_PER_STEP = 0.40   # rad — max wrist rotation per waypoint

BASE_FRAME  = 'reference/base'
EE_FRAME    = 'reference/right_hand'
TOOL_LENGTH = 0.212

# ─── Sensor bowl dimensions ────────────────────────────────────────────────
BOWL_OUTER = 0.80
BOWL_BASE  = 0.24
BOWL_RISE  = 0.09
BOWL_THICK = 0.005
BRKT_TALL  = 0.09
BRKT_SHORT = 0.05

# ─── Controller mapping (confirmed indices) ────────────────────────────────
BTN_A      = 0    # startup: go to HOME
BTN_B      = 1    # startup: skip home
BTN_X      = 2    # reset orientation to home
BTN_Y      = 3    # toggle orientation mode
BTN_L      = 4    # hold to ENABLE all motion
BTN_R      = 5    # toggle gripper
BTN_SELECT = 6    # re-anchor IK
BTN_START  = 7    # return to HOME
BTN_MODE   = 8    # (spare)
BTN_LS     = 9    # left stick click  (spare)
BTN_RS     = 10   # right stick click (spare)

AXIS_LX = 0   # left stick horizontal  → robot Y
AXIS_LY = 1   # left stick vertical    → robot X  (inverted below)
AXIS_RX = 3   # right stick horizontal → yaw / roll
AXIS_RY = 4   # right stick vertical   → Z / pitch (inverted below)

STICK_DEADZONE = 0.10   # ignore stick values below this

# Velocity scales (per second at full deflection)
XY_VEL_SCALE    = 0.15   # m/s
Z_VEL_SCALE     = 0.10   # m/s
YAW_VEL_SCALE   = 0.40   # rad/s
ROLL_VEL_SCALE  = 0.30   # rad/s
PITCH_VEL_SCALE = 0.30   # rad/s


# ─── Marker helpers ────────────────────────────────────────────────────────
def _sphere(ns, mid, r, g, b, size=0.025):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, mid
    m.type   = Marker.SPHERE
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = size
    m.color   = ColorRGBA(r, g, b, 1.0)
    m.pose.orientation.w = 1.0
    return m

def _text(ns, mid):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, mid
    m.type   = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.03
    m.color   = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.z = 0.4, 0.65
    return m

def _build_sensor_bowl_markers(cx, cy, cz):
    markers = []
    uid = 100
    GREY = ColorRGBA(0.45, 0.45, 0.50, 0.90)
    WOOD = ColorRGBA(0.87, 0.80, 0.60, 1.00)
    half_o = BOWL_OUTER / 2
    half_b = BOWL_BASE  / 2
    wt     = BOWL_THICK

    def cube(lx, ly, lz, sx, sy, sz, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id    = BASE_FRAME
        m.ns, m.id           = ns, uid;  uid += 1
        m.type               = Marker.CUBE
        m.action             = Marker.ADD
        m.pose.position.x    = float(cx + lx)
        m.pose.position.y    = float(cy + ly)
        m.pose.position.z    = float(cz + lz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = color
        return m

    def tri_list(tris, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id    = BASE_FRAME
        m.ns, m.id           = ns, uid;  uid += 1
        m.type               = Marker.TRIANGLE_LIST
        m.action             = Marker.ADD
        m.pose.position.x    = float(cx)
        m.pose.position.y    = float(cy)
        m.pose.position.z    = float(cz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = color
        for tri in tris:
            for v in tri:
                p = Point()
                p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
                m.points.append(p)
        return m

    markers.append(cube(0, 0, wt/2, BOWL_BASE, BOWL_BASE, wt, GREY, "bowl_base"))

    slope_defs = [
        dict(il=(-half_b,  half_b, 0), ir=( half_b,  half_b, 0),
             ol=(-half_o,  half_o, BOWL_RISE), or_=( half_o,  half_o, BOWL_RISE)),
        dict(il=(-half_b, -half_b, 0), ir=( half_b, -half_b, 0),
             ol=(-half_o, -half_o, BOWL_RISE), or_=( half_o, -half_o, BOWL_RISE)),
        dict(il=(-half_b, -half_b, 0), ir=(-half_b,  half_b, 0),
             ol=(-half_o, -half_o, BOWL_RISE), or_=(-half_o,  half_o, BOWL_RISE)),
        dict(il=( half_b, -half_b, 0), ir=( half_b,  half_b, 0),
             ol=( half_o, -half_o, BOWL_RISE), or_=( half_o,  half_o, BOWL_RISE)),
    ]
    tris = []
    for d in slope_defs:
        il, ir, ol, or_ = d['il'], d['ir'], d['ol'], d['or_']
        tris.append((il, ir, ol))
        tris.append((ir, or_, ol))
    markers.append(tri_list(tris, GREY, "bowl_slopes"))

    rim_lip_z = BOWL_RISE + wt / 2
    for cfg in [
        dict(lx=0,       ly= half_o, lz=rim_lip_z, sx=BOWL_OUTER, sy=wt, sz=wt),
        dict(lx=0,       ly=-half_o, lz=rim_lip_z, sx=BOWL_OUTER, sy=wt, sz=wt),
        dict(lx=-half_o, ly=0,       lz=rim_lip_z, sx=wt, sy=BOWL_OUTER, sz=wt),
        dict(lx= half_o, ly=0,       lz=rim_lip_z, sx=wt, sy=BOWL_OUTER, sz=wt),
    ]:
        markers.append(cube(cfg['lx'], cfg['ly'], cfg['lz'],
                            cfg['sx'], cfg['sy'], cfg['sz'], GREY, "bowl_rim"))

    brkt_tris = []
    for (bx, by, idx, _) in [
        ( half_o,  half_o, -1, 0),
        (-half_o,  half_o, +1, 0),
        ( half_o, -half_o, -1, 0),
        (-half_o, -half_o, +1, 0),
    ]:
        brkt_tris.append(((bx, by, 0.0), (bx, by, BRKT_TALL),
                          (bx + idx*BRKT_SHORT, by, 0.0)))
    markers.append(tri_list(brkt_tris, WOOD, "bowl_brackets"))
    return markers


# ─── Main node ─────────────────────────────────────────────────────────────
class PS2TeleopVizNode:
    def __init__(self):
        rospy.init_node('ps2_sawyer_teleop_viz', anonymous=False)

        self.control_rate     = rospy.get_param('~control_rate',     50.0)
        self.workspace_centre = rospy.get_param('~workspace_centre', [0.70, 0.0, 0.20])

        _sbp = rospy.get_param('~sensor_bowl_pos', [0.70, 0.0, 0.05])
        if isinstance(_sbp, str):
            import ast; _sbp = ast.literal_eval(_sbp)
        self._sensor_bowl_pos = [float(v) for v in _sbp]

        self.HOME_CONFIG = list(DEFAULT_HOME_CONFIG)

        # ── pygame ───────────────────────────────────────────────────────
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            rospy.logfatal("[ctrl] No joystick detected!")
            sys.exit(1)
        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        rospy.loginfo("[ctrl] %s  axes=%d  buttons=%d",
                      self._joy.get_name(),
                      self._joy.get_numaxes(),
                      self._joy.get_numbuttons())
        self._prev_btn = {}

        # ── TF ───────────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf)

        # ── Joint state publisher ────────────────────────────────────────
        self._js_pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
        self._current_angles = list(self.HOME_CONFIG)

        # ── RelaxedIK ────────────────────────────────────────────────────
        rospy.loginfo("[ctrl] Loading RelaxedIK...")
        saved = os.getcwd()
        os.chdir(_RIK_ROOT)
        try:
            self.rik = RelaxedIKRust(setting_file_path=_RIK_SETTINGS)
        finally:
            os.chdir(saved)
        rospy.loginfo("[ctrl] RelaxedIK OK")

        # ── Enable robot safety system ───────────────────────────────
        rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        rs.enable()
        rospy.loginfo("[ctrl] Robot enabled")

        self._limb = intera_interface.Limb('right')

        # ── Gripper ──────────────────────────────────────────────────────
        try:
            from pyrobotiqgripper import RobotiqGripper
            self._gripper       = RobotiqGripper()
            self._gripper.activate()
            rospy.sleep(0.5)
            self._gripper_ready = True
            rospy.loginfo("[ctrl] Gripper activated")
        except Exception as e:
            self._gripper_ready = False
            rospy.logwarn("[ctrl] Gripper init failed: %s", e)
        self._gripper_open = False

        # ── Markers ──────────────────────────────────────────────────────
        self._mk_pub    = rospy.Publisher('/teleop_viz', Marker, queue_size=20)
        self._mk_goal   = _sphere("goal",   0, 0.0, 1.0, 0.0, 0.03)
        self._mk_actual = _sphere("actual", 1, 1.0, 0.0, 0.0, 0.03)
        self._mk_info   = _text("info", 4)

        self._mk_box = Marker()
        self._mk_box.header.frame_id = BASE_FRAME
        self._mk_box.ns, self._mk_box.id = "virtual_box_fill", 10
        self._mk_box.type   = Marker.CUBE
        self._mk_box.action = Marker.ADD
        self._mk_box.scale.x = 0.80
        self._mk_box.scale.y = 0.80
        self._mk_box.scale.z = 0.40
        self._mk_box.color   = ColorRGBA(1.0, 0.5, 0.0, 0.15)
        self._mk_box.pose.orientation.w = 1.0

        self._mk_box_edges = Marker()
        self._mk_box_edges.header.frame_id = BASE_FRAME
        self._mk_box_edges.ns, self._mk_box_edges.id = "virtual_box_edges", 11
        self._mk_box_edges.type   = Marker.LINE_LIST
        self._mk_box_edges.action = Marker.ADD
        self._mk_box_edges.scale.x = 0.005
        self._mk_box_edges.color   = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        self._mk_box_edges.pose.orientation.w = 1.0

        self._mk_table = Marker()
        self._mk_table.header.frame_id = BASE_FRAME
        self._mk_table.ns, self._mk_table.id = "table", 20
        self._mk_table.type   = Marker.CUBE
        self._mk_table.action = Marker.ADD
        self._mk_table.scale.x = 1.80
        self._mk_table.scale.y = 1.20
        self._mk_table.scale.z = 0.05
        self._mk_table.color   = ColorRGBA(0.55, 0.40, 0.25, 1.0)
        self._mk_table.pose.orientation.w = 1.0

        self._mk_legs = []
        for i in range(4):
            leg = Marker()
            leg.header.frame_id = BASE_FRAME
            leg.ns, leg.id = "table_leg", 30 + i
            leg.type   = Marker.CYLINDER
            leg.action = Marker.ADD
            leg.scale.x = leg.scale.y = 0.06
            leg.scale.z = 0.80
            leg.color   = ColorRGBA(0.55, 0.40, 0.25, 1.0)
            leg.pose.orientation.w = 1.0
            self._mk_legs.append(leg)

        self._mk_wood_box = Marker()
        self._mk_wood_box.header.frame_id = BASE_FRAME
        self._mk_wood_box.ns, self._mk_wood_box.id = "wood_box", 40
        self._mk_wood_box.type   = Marker.CUBE
        self._mk_wood_box.action = Marker.ADD
        self._mk_wood_box.scale.x = 0.24
        self._mk_wood_box.scale.y = 0.24
        self._mk_wood_box.scale.z = 0.05
        self._mk_wood_box.color   = ColorRGBA(0.82, 0.65, 0.40, 1.0)
        self._mk_wood_box.pose.orientation.w = 1.0

        # ── Teleop state ─────────────────────────────────────────────────
        self._enabled        = False
        self._current_goal   = None
        self._home_pos       = None
        self._home_quat      = None

        # Orientation: stored as roll/pitch/yaw (integrated separately)
        self._ori_roll  = 0.0
        self._ori_pitch = 0.0
        self._ori_yaw   = 0.0

        # Mode 0 = yaw only (right stick X → yaw, right stick Y → Z)
        # Mode 1 = roll+pitch (right stick X → roll, right stick Y → pitch)
        self._ori_mode  = 0

    # ── Low-level helpers ──────────────────────────────────────────────────
    def _axis(self, idx):
        v = self._joy.get_axis(idx)
        return v if abs(v) > STICK_DEADZONE else 0.0

    def _btn_pressed(self, idx):
        """Rising-edge detection (single shot per press)."""
        cur  = bool(self._joy.get_button(idx))
        prev = self._prev_btn.get(idx, False)
        self._prev_btn[idx] = cur
        return cur and not prev

    def _btn_held(self, idx):
        return bool(self._joy.get_button(idx))

    def _publish_joint_states(self, angles, command_robot=True):
        v = GRIPPER_OPEN if self._gripper_open else GRIPPER_CLOSE
        gripper_pos = [v, v, v, v, -v, -v]
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES + GRIPPER_JOINT_NAMES
        msg.position = list(angles) + gripper_pos
        msg.velocity = [0.0] * (7 + len(GRIPPER_JOINT_NAMES))
        msg.effort   = [0.0] * (7 + len(GRIPPER_JOINT_NAMES))
        self._js_pub.publish(msg)
        if command_robot:
            self._limb.set_joint_positions(dict(zip(JOINT_NAMES, angles)))
        self._current_angles = list(angles)

    def _lookup_ee(self):
        try:
            t  = self._tf_buf.lookup_transform(
                BASE_FRAME, EE_FRAME, rospy.Time(0), rospy.Duration(1.0))
            tr = t.transform.translation
            ro = t.transform.rotation
            return [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w]
        except Exception:
            return None, None

    # ── Safe homing ────────────────────────────────────────────────────────
    def _move_to_home(self):
        rospy.loginfo("[ctrl] Reading current joint angles...")
        cur = self._limb.joint_angles()
        current = [cur.get(n, self.HOME_CONFIG[i]) for i, n in enumerate(JOINT_NAMES)]

        rospy.loginfo("[ctrl] Current: %s", [f"{a:.3f}" for a in current])
        rospy.loginfo("[ctrl] Target:  %s", [f"{a:.3f}" for a in self.HOME_CONFIG])

        j6_delta = abs(self.HOME_CONFIG[6] - current[6])
        n_steps  = max(SAFE_HOME_STEPS,
                       math.ceil(j6_delta / J6_MAX_DELTA_PER_STEP) + 1)
        step_timeout = SAFE_HOME_TIMEOUT / n_steps

        rospy.loginfo("[ctrl] Homing: %d steps, j6 travel=%.3f rad, speed=%.2f",
                      n_steps, j6_delta, SAFE_HOME_SPEED)

        self._limb.set_joint_position_speed(SAFE_HOME_SPEED)
        for step in range(1, n_steps + 1):
            if rospy.is_shutdown():
                return
            alpha    = step / n_steps
            waypoint = {}
            for i, name in enumerate(JOINT_NAMES):
                target = current[i] + alpha * (self.HOME_CONFIG[i] - current[i])
                if step == n_steps or abs(target - current[i]) >= SAFE_HOME_STEP_THRESH:
                    waypoint[name] = target
            if not waypoint:
                continue
            rospy.loginfo("[ctrl] Step %d/%d  j6=%.3f", step, n_steps,
                          waypoint.get(JOINT_NAMES[6],
                                       current[6] + alpha*(self.HOME_CONFIG[6]-current[6])))
            try:
                self._limb.move_to_joint_positions(waypoint, timeout=step_timeout)
            except Exception as e:
                rospy.logwarn("[ctrl] Waypoint %d failed: %s", step, e)

        rospy.loginfo("[ctrl] HOME reached safely")
        self._limb.set_joint_position_speed(1.0)   # restore full speed for teleop

    def _reset_orientation_to_home(self):
        """Snap the integrated roll/pitch/yaw back to the home orientation."""
        if self._home_quat is not None:
            r, p, y = tft.euler_from_quaternion(self._home_quat)
            self._ori_roll  = r
            self._ori_pitch = p
            self._ori_yaw   = y
            rospy.loginfo("[ctrl] Orientation reset to home (r=%.3f p=%.3f y=%.3f)", r, p, y)

    # ── Marker publishing ──────────────────────────────────────────────────
    def _publish_box(self):
        hx, hy, _ = self._home_pos
        x0, x1 = hx - 0.40, hx + 0.40
        y0, y1 = hy - 0.40, hy + 0.40
        z0 = float(self._sensor_bowl_pos[2])
        z1 = z0 + 0.40

        self._mk_box.header.stamp = rospy.Time.now()
        self._mk_box.pose.position.x = hx
        self._mk_box.pose.position.y = hy
        self._mk_box.pose.position.z = z0 + 0.20
        self._mk_pub.publish(self._mk_box)

        c = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
             (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
        edges = [(0,1),(1,2),(2,3),(3,0),(0,4),(1,5),(2,6),(3,7)]
        self._mk_box_edges.header.stamp = rospy.Time.now()
        self._mk_box_edges.points = []
        for a, b in edges:
            self._mk_box_edges.points.append(Point(*c[a]))
            self._mk_box_edges.points.append(Point(*c[b]))
        self._mk_pub.publish(self._mk_box_edges)

    def _publish_sensor_bowl(self):
        now = rospy.Time.now()
        hx, hy, _ = self._home_pos
        cz = float(self._sensor_bowl_pos[2])
        for m in _build_sensor_bowl_markers(hx, hy, cz):
            m.header.stamp = now
            self._mk_pub.publish(m)

    def _publish_table(self):
        hx, hy, _ = self._home_pos
        now = rospy.Time.now()
        table_top_z = 0.0
        thickness   = 0.05

        self._mk_table.header.stamp = now
        self._mk_table.pose.position.x = hx
        self._mk_table.pose.position.y = hy
        self._mk_table.pose.position.z = table_top_z - thickness / 2.0
        self._mk_pub.publish(self._mk_table)

        leg_z = table_top_z - thickness - 0.40
        for leg, (ox, oy) in zip(self._mk_legs,
                                  [(0.80,0.55),(0.80,-0.55),(-0.80,0.55),(-0.80,-0.55)]):
            leg.header.stamp = now
            leg.pose.position.x = hx + ox
            leg.pose.position.y = hy + oy
            leg.pose.position.z = leg_z
            self._mk_pub.publish(leg)

        self._mk_wood_box.header.stamp = now
        self._mk_wood_box.pose.position.x = hx
        self._mk_wood_box.pose.position.y = hy
        self._mk_wood_box.pose.position.z = table_top_z + self._mk_wood_box.scale.z / 2.0
        self._mk_pub.publish(self._mk_wood_box)

    def _publish_markers(self, goal_pos, actual_pos, enabled):
        now = rospy.Time.now()
        mode_str = "YAW-only" if self._ori_mode == 0 else "ROLL+PITCH"
        if goal_pos:
            self._mk_goal.header.stamp = now
            self._mk_goal.pose.position.x = goal_pos[0]
            self._mk_goal.pose.position.y = goal_pos[1]
            self._mk_goal.pose.position.z = goal_pos[2]
            self._mk_pub.publish(self._mk_goal)
        if actual_pos:
            self._mk_actual.header.stamp = now
            self._mk_actual.pose.position.x = actual_pos[0]
            self._mk_actual.pose.position.y = actual_pos[1]
            self._mk_actual.pose.position.z = actual_pos[2] - TOOL_LENGTH
            self._mk_pub.publish(self._mk_actual)
        if goal_pos and actual_pos:
            err = math.sqrt(sum((a-b)**2 for a,b in zip(goal_pos, actual_pos)))
            self._mk_info.header.stamp = now
            self._mk_info.text = (
                f"{'ENABLED' if enabled else 'DISABLED'}  [{mode_str}]\n"
                f"Goal:   [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]\n"
                f"Actual: [{actual_pos[0]:.3f}, {actual_pos[1]:.3f}, {actual_pos[2]:.3f}]\n"
                f"Error:  {err*1000:.1f} mm\n"
                f"RPY:    [{math.degrees(self._ori_roll):.1f}°, "
                f"{math.degrees(self._ori_pitch):.1f}°, "
                f"{math.degrees(self._ori_yaw):.1f}°]\n"
                f"Joints: [{', '.join(f'{a:.2f}' for a in self._current_angles)}]"
            )
            self._mk_pub.publish(self._mk_info)

    # ── Entry point ────────────────────────────────────────────────────────
    def run(self):
        import termios
        old_term = termios.tcgetattr(sys.stdin)
        try:
            self._run_inner()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)

    def _run_inner(self):
        import select, termios, tty as _tty
        _tty.setcbreak(sys.stdin.fileno())

        # ── Read current joints for display ──────────────────────────────
        cur_dict   = self._limb.joint_angles()
        cur_angles = [cur_dict.get(n, self.HOME_CONFIG[i])
                      for i, n in enumerate(JOINT_NAMES)]

        print("\n" + "="*62)
        print("  Sawyer Gamepad Teleop — startup")
        print("="*62)
        print(f"  Current joints: {[f'{a:.3f}' for a in cur_angles]}")
        print(f"  Home    joints: {[f'{a:.3f}' for a in self.HOME_CONFIG]}")
        print()
        print("  Press on the controller:")
        print("    [A]  →  move to HOME first  (safe, slow)")
        print("    [B]  →  skip home, start from current position")
        print("="*62 + "\n")

        pygame.event.clear()
        go_home = None
        while go_home is None and not rospy.is_shutdown():
            pygame.event.pump()
            if self._joy.get_button(BTN_A):
                go_home = True
                print("  [A] pressed — moving to HOME safely...\n")
            elif self._joy.get_button(BTN_B):
                go_home = False
                print("  [B] pressed — starting from current position\n")
            rospy.sleep(0.05)

        if go_home:
            self._move_to_home()
            warmup_angles = list(self.HOME_CONFIG)
        else:
            rospy.loginfo("[ctrl] Skipping home — seeding IK from current angles")
            self.rik.reset(cur_angles)
            self._current_angles = cur_angles
            warmup_angles = cur_angles

        # ── TF warmup ────────────────────────────────────────────────────
        rospy.loginfo("[ctrl] Warming up TF...")
        r = rospy.Rate(50)
        for _ in range(100):
            self._publish_joint_states(warmup_angles, command_robot=False)
            r.sleep()

        pos, quat = self._lookup_ee()
        if pos is None:
            rospy.logfatal("[ctrl] Cannot read EE TF — is the robot connected?")
            return

        self._home_pos  = pos
        self._home_quat = quat
        rospy.loginfo("[ctrl] EE start: [%.4f, %.4f, %.4f]", *pos)

        self._current_goal = list(pos)
        self._reset_orientation_to_home()   # seed RPY from actual home quat

        print("\n" + "="*62)
        print("  CONTROLS")
        print("="*62)
        print("  Hold [L]          → enable motion")
        print("  Left  stick       → X / Y position")
        print("  Right stick Y     → Z position (up/down)")
        print("  Right stick X     → YAW  (mode 0, default)")
        print("                      ROLL (mode 1)")
        print("  [Y]               → toggle orientation mode")
        print("                      mode 0: yaw-only")
        print("                      mode 1: roll + pitch")
        print("  [X]               → reset orientation to home")
        print("  [R]               → toggle gripper open/close")
        print("  [Start]           → return to HOME (safe)")
        print("  [Select]          → re-anchor IK at current pos")
        print("  keyboard 'r'      → same as Start")
        print("="*62 + "\n")

        dt   = 1.0 / self.control_rate
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            pygame.event.pump()

            # ── Keyboard shortcut ────────────────────────────────────────
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'r':
                    rospy.loginfo("[ctrl] 'r' — returning to HOME")
                    self._move_to_home()
                    self._current_goal = list(self._home_pos)
                    self._reset_orientation_to_home()
                    self._enabled        = False
                    self._current_angles = list(self.HOME_CONFIG)

            # ── [Start] → HOME ───────────────────────────────────────────
            if self._btn_pressed(BTN_START):
                rospy.loginfo("[ctrl] START — returning to HOME")
                self._move_to_home()
                self._current_goal   = list(self._home_pos)
                self._reset_orientation_to_home()
                self._enabled        = False
                self._current_angles = list(self.HOME_CONFIG)

            # ── [Select] → re-anchor ─────────────────────────────────────
            if self._btn_pressed(BTN_SELECT):
                self.rik.reset(list(self._current_angles))
                cur_pos, _ = self._lookup_ee()
                if cur_pos:
                    self._current_goal = cur_pos
                rospy.loginfo("[ctrl] SELECT — re-anchored at %.3f %.3f %.3f",
                              *self._current_goal)

            # ── [X] → reset orientation ──────────────────────────────────
            if self._btn_pressed(BTN_X):
                self._reset_orientation_to_home()
                rospy.loginfo("[ctrl] Orientation reset to home")

            # ── [Y] → toggle orientation mode ────────────────────────────
            if self._btn_pressed(BTN_Y):
                self._ori_mode = 1 - self._ori_mode
                mode_name = "ROLL+PITCH" if self._ori_mode == 1 else "YAW-only"
                rospy.loginfo("[ctrl] Orientation mode → %s", mode_name)
                print(f"  [Y] Orientation mode: {mode_name}")

            # ── [R] → gripper toggle ─────────────────────────────────────
            if self._btn_pressed(BTN_R) and self._gripper_ready:
                if self._gripper_open:
                    self._gripper.close()
                    self._gripper_open = False
                    rospy.loginfo("[ctrl] Gripper CLOSED")
                else:
                    self._gripper.open()
                    self._gripper_open = True
                    rospy.loginfo("[ctrl] Gripper OPEN")

            # ── [L] held → motion enabled ────────────────────────────────
            self._enabled = self._btn_held(BTN_L)
            if self._enabled:
                rospy.loginfo_throttle(1.0, "[ctrl] ENABLED — L is held")

            goal_pos = None

            if self._enabled:
                # ── Position ─────────────────────────────────────────────
                lx =  self._axis(AXIS_LX)    # → robot Y
                ly = -self._axis(AXIS_LY)    # → robot X  (push fwd = +X)
                rx =  self._axis(AXIS_RX)    # → yaw or roll
                ry = -self._axis(AXIS_RY)    # → Z (push fwd = +Z) or pitch
                rospy.loginfo_throttle(0.5, "[ctrl] axes raw LX=%.3f LY=%.3f RX=%.3f RY=%.3f",
                    self._joy.get_axis(AXIS_LX), self._joy.get_axis(AXIS_LY),
                    self._joy.get_axis(AXIS_RX), self._joy.get_axis(AXIS_RY))

                gx = self._current_goal[0] + ly * XY_VEL_SCALE * dt
                gy = self._current_goal[1] + lx * XY_VEL_SCALE * dt

                # ── Orientation + Z depend on mode ────────────────────────
                if self._ori_mode == 0:
                    # Mode 0: right stick X → yaw,  right stick Y → Z
                    gz = self._current_goal[2] + ry * Z_VEL_SCALE * dt
                    self._ori_yaw += rx * YAW_VEL_SCALE * dt
                else:
                    # Mode 1: right stick X → roll, right stick Y → pitch
                    # Z is frozen (use mode 0 to change height)
                    gz = self._current_goal[2]
                    self._ori_roll  += rx * ROLL_VEL_SCALE  * dt
                    self._ori_pitch += ry * PITCH_VEL_SCALE * dt

                # ── Workspace clamping ───────────────────────────────────
                hx, hy, _ = self._home_pos
                bowl_floor = float(self._sensor_bowl_pos[2]) + TOOL_LENGTH

                if gx < hx - 0.40: gx = hx - 0.40; rospy.logwarn_throttle(1.0, "[box] X min")
                if gx > hx + 0.40: gx = hx + 0.40; rospy.logwarn_throttle(1.0, "[box] X max")
                if gy < hy - 0.40: gy = hy - 0.40; rospy.logwarn_throttle(1.0, "[box] Y min")
                if gy > hy + 0.40: gy = hy + 0.40; rospy.logwarn_throttle(1.0, "[box] Y max")
                if gz < bowl_floor: gz = bowl_floor; rospy.logwarn_throttle(0.5,
                    "[box] Z floor gz=%.4f", gz)

                self._current_goal = [gx, gy, gz]
                goal_pos = self._current_goal

                # ── Build goal quaternion from RPY ───────────────────────
                goal_quat = list(tft.quaternion_from_euler(
                    self._ori_roll, self._ori_pitch, self._ori_yaw))

                # ── IK solve ─────────────────────────────────────────────
                try:
                    angles = self.rik.solve_position(
                        positions=goal_pos,
                        orientations=goal_quat,
                        tolerances=[0.0] * 6)
                    if len(angles) == 7 and all(math.isfinite(a) for a in angles):
                        self._publish_joint_states(angles)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "[ctrl] IK failed: %s", e)
            else:
                self._publish_joint_states(self._current_angles)

            actual_pos, _ = self._lookup_ee()
            self._publish_markers(goal_pos or self._current_goal, actual_pos, self._enabled)
            self._publish_box()
            self._publish_sensor_bowl()
            self._publish_table()
            rate.sleep()


def main():
    node = PS2TeleopVizNode()
    node.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass