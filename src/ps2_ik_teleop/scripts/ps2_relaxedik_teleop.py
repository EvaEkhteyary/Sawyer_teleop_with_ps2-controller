#!/usr/bin/env python3
"""
ps2_sawyer_teleop_viz.py — PS2 Controller teleoperation + RViz visualisation
=============================================================================
Safer PS2 version of the old TouchX teleop node.

Current mapping used:
  Left stick X/Y  -> robot Y/X
  Right stick Y   -> robot Z
  Right stick X   -> disabled for now
  L1              -> hold to ENABLE teleoperation
  R1              -> toggle gripper open/close
  Start           -> return to HOME
  Select          -> reset RelaxedIK / re-anchor
  Keyboard 'r'    -> return to HOME

Notes:
- Yaw is disabled because one of the reported controller axes was not centred.
- Motion speeds are reduced for safer testing.
- Z has both floor and ceiling clamping.
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


# ─── Robot constants ────────────────────────────────────────────────────────
JOINT_NAMES = [
    'right_j0', 'right_j1', 'right_j2',
    'right_j3', 'right_j4', 'right_j5', 'right_j6'
]

GRIPPER_JOINT_NAMES = [
    'finger_joint',
    'right_outer_knuckle_joint',
    'left_inner_knuckle_joint', 'right_inner_knuckle_joint',
    'left_inner_finger_joint',  'right_inner_finger_joint',
]

GRIPPER_OPEN        = 0.0
GRIPPER_CLOSE       = 0.8
DEFAULT_HOME_CONFIG = [0.3474, -1.3143, -0.5663, 1.3630, 0.0967, 1.4469, 3.0276]

BASE_FRAME  = 'reference/base'
EE_FRAME    = 'reference/right_hand'
TOOL_LENGTH = 0.212


# ─── Sensor bowl dimensions ────────────────────────────────────────────────
BOWL_OUTER  = 0.80
BOWL_BASE   = 0.24
BOWL_RISE   = 0.09
BOWL_THICK  = 0.005
BRKT_TALL   = 0.09
BRKT_SHORT  = 0.05


# ─── PS2 controller mapping ────────────────────────────────────────────────
PS2_AXIS_LX    = 0      # safe
PS2_AXIS_LY    = 1      # safe
PS2_AXIS_RY    = 3      # likely safe for Z
PS2_AXIS_RX    = None   # disable yaw for now

PS2_BTN_L1     = 4
PS2_BTN_R1     = 5
PS2_BTN_SELECT = 6
PS2_BTN_START  = 7

STICK_DEADZONE = 0.20
XY_VEL_SCALE   = 0.05   # m/s
Z_VEL_SCALE    = 0.03   # m/s
YAW_VEL_SCALE  = 0.0    # disabled


# ─── Marker helpers ────────────────────────────────────────────────────────
def _sphere(ns, mid, r, g, b, size=0.025):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = size
    m.color = ColorRGBA(r, g, b, 1.0)
    m.pose.orientation.w = 1.0
    return m


def _text(ns, mid):
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, mid
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.03
    m.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.z = 0.4, 0.65
    return m


def _build_sensor_bowl_markers(cx, cy, cz):
    markers = []
    uid = 100

    GREY = ColorRGBA(0.45, 0.45, 0.50, 0.90)
    WOOD = ColorRGBA(0.87, 0.80, 0.60, 1.00)

    half_o = BOWL_OUTER / 2
    half_b = BOWL_BASE / 2
    wt = BOWL_THICK

    def cube(lx, ly, lz, sx, sy, sz, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.ns, m.id = ns, uid
        uid += 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(cx + lx)
        m.pose.position.y = float(cy + ly)
        m.pose.position.z = float(cz + lz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = color
        return m

    def tri_list(tris, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.ns, m.id = ns, uid
        uid += 1
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.position.x = float(cx)
        m.pose.position.y = float(cy)
        m.pose.position.z = float(cz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = color
        for tri in tris:
            for v in tri:
                p = Point()
                p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
                m.points.append(p)
        return m

    markers.append(cube(0, 0, wt / 2, BOWL_BASE, BOWL_BASE, wt, GREY, "bowl_base"))

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
        markers.append(cube(
            cfg['lx'], cfg['ly'], cfg['lz'],
            cfg['sx'], cfg['sy'], cfg['sz'],
            GREY, "bowl_rim"
        ))

    brkt_tris = []
    for (bx, by, idx, _) in [
        ( half_o,  half_o, -1, 0),
        (-half_o,  half_o, +1, 0),
        ( half_o, -half_o, -1, 0),
        (-half_o, -half_o, +1, 0),
    ]:
        p_bot  = (bx, by, 0.0)
        p_top  = (bx, by, BRKT_TALL)
        p_foot = (bx + idx * BRKT_SHORT, by, 0.0)
        brkt_tris.append((p_bot, p_top, p_foot))
    markers.append(tri_list(brkt_tris, WOOD, "bowl_brackets"))

    return markers


# ─── Main node ─────────────────────────────────────────────────────────────
class PS2TeleopVizNode:
    def __init__(self):
        rospy.init_node('ps2_sawyer_teleop_viz', anonymous=False)

        self.control_rate     = rospy.get_param('~control_rate', 50.0)
        self.workspace_centre = rospy.get_param('~workspace_centre', [0.70, 0.0, 0.20])

        _sbp = rospy.get_param('~sensor_bowl_pos', [0.70, 0.0, 0.05])
        if isinstance(_sbp, str):
            import ast
            _sbp = ast.literal_eval(_sbp)
        self._sensor_bowl_pos = [float(v) for v in _sbp]

        self.HOME_CONFIG = list(DEFAULT_HOME_CONFIG)

        # pygame joystick init
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            rospy.logfatal("[ps2] No joystick detected — is the controller plugged in?")
            sys.exit(1)

        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        rospy.loginfo(
            "[ps2] Controller: %s  axes=%d  buttons=%d",
            self._joy.get_name(),
            self._joy.get_numaxes(),
            self._joy.get_numbuttons()
        )

        self._prev_btn = {}

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf)

        # Joint state publisher
        self._js_pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
        self._current_angles = list(self.HOME_CONFIG)

        # RelaxedIK
        rospy.loginfo("[ps2] Loading RelaxedIK...")
        saved = os.getcwd()
        os.chdir(_RIK_ROOT)
        try:
            self.rik = RelaxedIKRust(setting_file_path=_RIK_SETTINGS)
        finally:
            os.chdir(saved)
        rospy.loginfo("[ps2] RelaxedIK OK")

        self._limb = intera_interface.Limb('right')

        # Gripper
        try:
            from pyrobotiqgripper import RobotiqGripper
            self._gripper = RobotiqGripper()
            self._gripper.activate()
            rospy.sleep(0.5)
            self._gripper_ready = True
            rospy.loginfo("[ps2] Gripper activated")
        except Exception as e:
            self._gripper_ready = False
            rospy.logwarn("[ps2] Gripper init failed: %s", e)

        self._gripper_open = False

        # Markers
        self._mk_pub    = rospy.Publisher('/teleop_viz', Marker, queue_size=20)
        self._mk_goal   = _sphere("goal",   0, 0.0, 1.0, 0.0, 0.03)
        self._mk_actual = _sphere("actual", 1, 1.0, 0.0, 0.0, 0.03)
        self._mk_info   = _text("info", 4)

        self._mk_box = Marker()
        self._mk_box.header.frame_id = BASE_FRAME
        self._mk_box.ns, self._mk_box.id = "virtual_box_fill", 10
        self._mk_box.type = Marker.CUBE
        self._mk_box.action = Marker.ADD
        self._mk_box.scale.x = 0.80
        self._mk_box.scale.y = 0.80
        self._mk_box.scale.z = 0.40
        self._mk_box.color = ColorRGBA(1.0, 0.5, 0.0, 0.15)
        self._mk_box.pose.orientation.w = 1.0

        self._mk_table = Marker()
        self._mk_table.header.frame_id = BASE_FRAME
        self._mk_table.ns, self._mk_table.id = "table", 20
        self._mk_table.type = Marker.CUBE
        self._mk_table.action = Marker.ADD
        self._mk_table.scale.x = 1.80
        self._mk_table.scale.y = 1.20
        self._mk_table.scale.z = 0.05
        self._mk_table.color = ColorRGBA(0.55, 0.40, 0.25, 1.0)
        self._mk_table.pose.orientation.w = 1.0

        self._mk_legs = []
        for i in range(4):
            leg = Marker()
            leg.header.frame_id = BASE_FRAME
            leg.ns, leg.id = "table_leg", 30 + i
            leg.type = Marker.CYLINDER
            leg.action = Marker.ADD
            leg.scale.x = leg.scale.y = 0.06
            leg.scale.z = 0.80
            leg.color = ColorRGBA(0.55, 0.40, 0.25, 1.0)
            leg.pose.orientation.w = 1.0
            self._mk_legs.append(leg)

        self._mk_wood_box = Marker()
        self._mk_wood_box.header.frame_id = BASE_FRAME
        self._mk_wood_box.ns, self._mk_wood_box.id = "wood_box", 40
        self._mk_wood_box.type = Marker.CUBE
        self._mk_wood_box.action = Marker.ADD
        self._mk_wood_box.scale.x = 0.24
        self._mk_wood_box.scale.y = 0.24
        self._mk_wood_box.scale.z = 0.05
        self._mk_wood_box.color = ColorRGBA(0.82, 0.65, 0.40, 1.0)
        self._mk_wood_box.pose.orientation.w = 1.0

        self._mk_box_edges = Marker()
        self._mk_box_edges.header.frame_id = BASE_FRAME
        self._mk_box_edges.ns, self._mk_box_edges.id = "virtual_box_edges", 11
        self._mk_box_edges.type = Marker.LINE_LIST
        self._mk_box_edges.action = Marker.ADD
        self._mk_box_edges.scale.x = 0.005
        self._mk_box_edges.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        self._mk_box_edges.pose.orientation.w = 1.0

        # Teleop state
        self._enabled      = False
        self._current_goal = None
        self._current_yaw  = 0.0
        self._home_pos     = None
        self._home_quat    = None

    # ── Helpers ────────────────────────────────────────────────────────
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

    def _move_to_home(self):
        rospy.loginfo("[ps2] Moving to HOME position...")
        self._limb.move_to_joint_positions(
            dict(zip(JOINT_NAMES, self.HOME_CONFIG)),
            timeout=10.0
        )
        rospy.loginfo("[ps2] Reached HOME")

    def _lookup_ee(self):
        try:
            t = self._tf_buf.lookup_transform(
                BASE_FRAME, EE_FRAME, rospy.Time(0), rospy.Duration(1.0)
            )
            tr = t.transform.translation
            ro = t.transform.rotation
            return [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w]
        except Exception:
            return None, None

    def _axis(self, idx):
        if idx is None:
            return 0.0
        v = self._joy.get_axis(idx)
        if not math.isfinite(v):
            return 0.0
        return v if abs(v) > STICK_DEADZONE else 0.0

    def _btn_pressed(self, idx):
        cur = bool(self._joy.get_button(idx))
        prev = self._prev_btn.get(idx, False)
        self._prev_btn[idx] = cur
        return cur and not prev

    def _btn_held(self, idx):
        return bool(self._joy.get_button(idx))

    # ── Visualisation ─────────────────────────────────────────────────
    def _publish_box(self):
        hx, hy, _ = self._home_pos
        x0, x1 = hx - 0.40, hx + 0.40
        y0, y1 = hy - 0.40, hy + 0.40
        z0 = float(self._sensor_bowl_pos[2])
        z1 = z0 + 0.40
        box_centre_z = z0 + 0.20

        self._mk_box.header.stamp = rospy.Time.now()
        self._mk_box.pose.position.x = hx
        self._mk_box.pose.position.y = hy
        self._mk_box.pose.position.z = box_centre_z
        self._mk_pub.publish(self._mk_box)

        c = [
            (x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
            (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1),
        ]
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
        table_thickness = 0.05
        table_z_centre = table_top_z - (table_thickness / 2.0)

        self._mk_table.header.stamp = now
        self._mk_table.pose.position.x = hx
        self._mk_table.pose.position.y = hy
        self._mk_table.pose.position.z = table_z_centre
        self._mk_pub.publish(self._mk_table)

        leg_height = 0.80
        leg_z = table_top_z - table_thickness - (leg_height / 2.0)
        offsets = [( 0.80,  0.55), ( 0.80, -0.55),
                   (-0.80,  0.55), (-0.80, -0.55)]

        for leg, (ox, oy) in zip(self._mk_legs, offsets):
            leg.header.stamp = now
            leg.pose.position.x = hx + ox
            leg.pose.position.y = hy + oy
            leg.pose.position.z = leg_z
            self._mk_pub.publish(leg)

        box_height = self._mk_wood_box.scale.z
        box_z_centre = table_top_z + (box_height / 2.0)

        self._mk_wood_box.header.stamp = now
        self._mk_wood_box.pose.position.x = hx
        self._mk_wood_box.pose.position.y = hy
        self._mk_wood_box.pose.position.z = box_z_centre
        self._mk_pub.publish(self._mk_wood_box)

    def _publish_markers(self, goal_pos, actual_pos, enabled):
        now = rospy.Time.now()

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
            err = math.sqrt(sum((a - b) ** 2 for a, b in zip(goal_pos, actual_pos)))
            self._mk_info.header.stamp = now
            self._mk_info.text = (
                f"{'ENABLED' if enabled else 'DISABLED'}\n"
                f"Goal:   [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]\n"
                f"Actual: [{actual_pos[0]:.3f}, {actual_pos[1]:.3f}, {actual_pos[2]:.3f}]\n"
                f"Error:  {err * 1000:.1f} mm\n"
                f"Joints: [{', '.join(f'{a:.2f}' for a in self._current_angles)}]"
            )
            self._mk_pub.publish(self._mk_info)

    # ── Main loop ──────────────────────────────────────────────────────
    def run(self):
        import tty
        import termios

        old_term = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self._run_inner()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)

    def _run_inner(self):
        import select

        self._move_to_home()

        rospy.loginfo("[ps2] Warming up TF for RViz...")
        warm_rate = rospy.Rate(50)
        for _ in range(100):
            self._publish_joint_states(self.HOME_CONFIG, command_robot=False)
            warm_rate.sleep()

        pos, quat = self._lookup_ee()
        if pos is None:
            rospy.logfatal("[ps2] Cannot read EE TF. Is robot connected?")
            return

        self._home_pos  = pos
        self._home_quat = quat
        self._current_goal = list(pos)
        self._current_yaw = tft.euler_from_quaternion(quat)[2]

        rospy.loginfo("[ps2] HOME EE: [%.4f, %.4f, %.4f]", *pos)
        rospy.loginfo("[ps2] Ready.")
        rospy.loginfo("[ps2]   L1            = hold to ENABLE teleoperation")
        rospy.loginfo("[ps2]   R1            = toggle gripper open / close")
        rospy.loginfo("[ps2]   Start         = return to HOME")
        rospy.loginfo("[ps2]   Select        = re-anchor (reset IK)")
        rospy.loginfo("[ps2]   Left stick    = robot X/Y")
        rospy.loginfo("[ps2]   Right stick Y = robot Z")
        rospy.loginfo("[ps2]   Right stick X = yaw (disabled for now)")
        rospy.loginfo("[ps2]   Keyboard 'r'  = HOME")

        dt = 1.0 / self.control_rate
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            pygame.event.pump()

            # keyboard shortcut
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'r':
                    rospy.loginfo("[ps2] 'r' pressed — returning to HOME")
                    self._move_to_home()

                    cur_pos, cur_quat = self._lookup_ee()
                    if cur_pos is not None:
                        self._current_goal = list(cur_pos)
                    else:
                        self._current_goal = list(self._home_pos)

                    if cur_quat is not None:
                        self._current_yaw = tft.euler_from_quaternion(cur_quat)[2]
                    else:
                        self._current_yaw = tft.euler_from_quaternion(self._home_quat)[2]

                    self._enabled = False
                    self._current_angles = list(self.HOME_CONFIG)
                    self.rik.reset(list(self._current_angles))

            # START -> HOME
            if self._btn_pressed(PS2_BTN_START):
                rospy.loginfo("[ps2] START — returning to HOME")
                self._move_to_home()

                cur_pos, cur_quat = self._lookup_ee()
                if cur_pos is not None:
                    self._current_goal = list(cur_pos)
                else:
                    self._current_goal = list(self._home_pos)

                if cur_quat is not None:
                    self._current_yaw = tft.euler_from_quaternion(cur_quat)[2]
                else:
                    self._current_yaw = tft.euler_from_quaternion(self._home_quat)[2]

                self._enabled = False
                self._current_angles = list(self.HOME_CONFIG)
                self.rik.reset(list(self._current_angles))

            # SELECT -> re-anchor
            if self._btn_pressed(PS2_BTN_SELECT):
                self.rik.reset(list(self._current_angles))
                cur_pos, _ = self._lookup_ee()
                if cur_pos is not None:
                    self._current_goal = list(cur_pos)
                rospy.loginfo("[ps2] SELECT — re-anchored at %.3f %.3f %.3f",
                              *self._current_goal)

            # R1 -> gripper toggle
            if self._btn_pressed(PS2_BTN_R1) and self._gripper_ready:
                if self._gripper_open:
                    self._gripper.close()
                    self._gripper_open = False
                    rospy.loginfo("[ps2] Gripper CLOSED")
                else:
                    self._gripper.open()
                    self._gripper_open = True
                    rospy.loginfo("[ps2] Gripper OPEN")

            # L1 hold -> enable teleop
            self._enabled = self._btn_held(PS2_BTN_L1) and (self._current_goal is not None)

            if self._enabled:
                rospy.loginfo_throttle(1.0, "[ps2] Teleop ENABLED")

            goal_pos = None

            if self._enabled:
                lx =  self._axis(PS2_AXIS_LX)
                ly = -self._axis(PS2_AXIS_LY)
                rx = 0.0
                rz = -self._axis(PS2_AXIS_RY)

                rospy.loginfo_throttle(0.5, "[ps2] lx=%.3f ly=%.3f rz=%.3f", lx, ly, rz)

                gx = self._current_goal[0] + ly * XY_VEL_SCALE * dt
                gy = self._current_goal[1] + lx * XY_VEL_SCALE * dt
                gz = self._current_goal[2] + rz * Z_VEL_SCALE * dt

                self._current_yaw += rx * YAW_VEL_SCALE * dt

                hx, hy, _ = self._home_pos
                bowl_floor = float(self._sensor_bowl_pos[2]) + TOOL_LENGTH
                z_ceiling  = bowl_floor + 0.25

                if gx < hx - 0.25:
                    gx = hx - 0.25
                    rospy.logwarn_throttle(1.0, "[box] Hit X min wall")
                if gx > hx + 0.25:
                    gx = hx + 0.25
                    rospy.logwarn_throttle(1.0, "[box] Hit X max wall")

                if gy < hy - 0.25:
                    gy = hy - 0.25
                    rospy.logwarn_throttle(1.0, "[box] Hit Y min wall")
                if gy > hy + 0.25:
                    gy = hy + 0.25
                    rospy.logwarn_throttle(1.0, "[box] Hit Y max wall")

                if gz < bowl_floor:
                    gz = bowl_floor
                    rospy.logwarn_throttle(0.5, "[box] Hit Z FLOOR gz=%.4f", gz)
                if gz > z_ceiling:
                    gz = z_ceiling
                    rospy.logwarn_throttle(0.5, "[box] Hit Z CEILING gz=%.4f", gz)

                self._current_goal = [gx, gy, gz]
                goal_pos = self._current_goal

                home_r, home_p, _ = tft.euler_from_quaternion(self._home_quat)
                goal_quat = list(tft.quaternion_from_euler(home_r, home_p, self._current_yaw))

                try:
                    angles = self.rik.solve_position(
                        positions=goal_pos,
                        orientations=goal_quat,
                        tolerances=[0.0] * 6
                    )
                    if len(angles) == 7 and all(math.isfinite(a) for a in angles):
                        self._publish_joint_states(angles)
                except Exception as e:
                    rospy.logwarn_throttle(2.0, "[ps2] IK failed: %s", e)
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