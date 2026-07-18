#!/usr/bin/env python3
"""
Action CLI — unified terminal interface for G1 right arm + Wuji Hand 2 (Plan C).

  Position (XYZ)   → /target_point  → reach_node (4-DOF arm IK)
  Orientation+grasp → /hand_command  → hand_controller (23-DOF)

Usage:
  python3 action_cli.py reach <x> <y> <z> [--ori qx qy qz qw]
  python3 action_cli.py grasp <pattern> [--duration 0.08]
  python3 action_cli.py reach_and_grasp <x> <y> <z> <pattern> [--ori ...]
  python3 action_cli.py home | open | list | interactive

All commands publish to /action_command, consumed by ActionSequencerNode.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from g1_right_arm_reach.msg import GraspCommand, ActionCommand


# ===========================================================================
# CLI color helpers
# ===========================================================================
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


def ok(msg):
    print(f"{Colors.GREEN}[OK]{Colors.RESET} {msg}")


def warn(msg):
    print(f"{Colors.YELLOW}[WARN]{Colors.RESET} {msg}")


def info(msg):
    print(f"{Colors.CYAN}[INFO]{Colors.RESET} {msg}")


def err(msg):
    print(f"{Colors.RED}[ERROR]{Colors.RESET} {msg}")
    sys.exit(1)


# ===========================================================================
# Command implementations
# ===========================================================================
def cmd_reach(args):
    """Publish /action_command with action_type=reach."""
    rclpy.init(args=[])
    node = Node('action_cli_reach')
    pub = node.create_publisher(ActionCommand, '/action_command', 10)

    cmd = ActionCommand()
    cmd.action_type = 'reach'
    cmd.target_x = args.x
    cmd.target_y = args.y
    cmd.target_z = args.z
    if args.ori:
        cmd.ori_x, cmd.ori_y, cmd.ori_z, cmd.ori_w = args.ori
    else:
        cmd.ori_w = 1.0
    cmd.convergence_timeout = args.timeout
    cmd.settle_time = args.settle

    time.sleep(0.2)  # Let ROS discovery complete
    pub.publish(cmd)
    time.sleep(0.1)

    ori_str = f" ori=[{cmd.ori_x:.1f},{cmd.ori_y:.1f},{cmd.ori_z:.1f},{cmd.ori_w:.1f}]" if args.ori else ""
    ok(f"REACH → [{cmd.target_x:.2f}, {cmd.target_y:.2f}, {cmd.target_z:.2f}]{ori_str}  (timeout={args.timeout}s)")

    node.destroy_node()
    rclpy.shutdown()


def cmd_grasp(args):
    """Publish /action_command with action_type=grasp."""
    rclpy.init(args=[])
    node = Node('action_cli_grasp')
    pub = node.create_publisher(ActionCommand, '/action_command', 10)

    cmd = ActionCommand()
    cmd.action_type = 'grasp'
    cmd.grasp_type = args.pattern
    cmd.grasp_duration = args.duration

    time.sleep(0.2)
    pub.publish(cmd)
    time.sleep(0.1)

    ok(f"GRASP → {args.pattern}  (duration={args.duration}s)")

    node.destroy_node()
    rclpy.shutdown()


def cmd_reach_and_grasp(args):
    """Publish /action_command with action_type=reach_and_grasp."""
    rclpy.init(args=[])
    node = Node('action_cli_rng')
    pub = node.create_publisher(ActionCommand, '/action_command', 10)

    cmd = ActionCommand()
    cmd.action_type = 'reach_and_grasp'
    cmd.target_x = args.x
    cmd.target_y = args.y
    cmd.target_z = args.z
    if args.ori:
        cmd.ori_x, cmd.ori_y, cmd.ori_z, cmd.ori_w = args.ori
    else:
        cmd.ori_w = 1.0
    cmd.grasp_type = args.pattern
    cmd.grasp_duration = args.duration
    cmd.convergence_timeout = args.timeout
    cmd.settle_time = args.settle

    time.sleep(0.2)
    pub.publish(cmd)
    time.sleep(0.1)

    ori_str = f" ori=[{cmd.ori_x:.1f},{cmd.ori_y:.1f},{cmd.ori_z:.1f},{cmd.ori_w:.1f}]" if args.ori else ""
    ok(f"REACH_AND_GRASP → [{cmd.target_x:.2f}, {cmd.target_y:.2f}, {cmd.target_z:.2f}]{ori_str} → {args.pattern}  (duration={args.duration}s)")

    node.destroy_node()
    rclpy.shutdown()


def cmd_home(args):
    """Publish /action_command with action_type=home."""
    rclpy.init(args=[])
    node = Node('action_cli_home')
    pub = node.create_publisher(ActionCommand, '/action_command', 10)

    cmd = ActionCommand()
    cmd.action_type = 'home'

    time.sleep(0.2)
    pub.publish(cmd)
    time.sleep(0.1)

    ok("HOME → ready pose + home grasp")

    node.destroy_node()
    rclpy.shutdown()


def cmd_open(args):
    """Shortcut: publish grasp open."""
    rclpy.init(args=[])
    node = Node('action_cli_open')
    pub = node.create_publisher(ActionCommand, '/action_command', 10)

    cmd = ActionCommand()
    cmd.action_type = 'grasp'
    cmd.grasp_type = 'open'
    cmd.grasp_duration = 0.3

    time.sleep(0.2)
    pub.publish(cmd)
    time.sleep(0.1)

    ok("OPEN hand")

    node.destroy_node()
    rclpy.shutdown()


def cmd_list(args):
    """Print available grasp patterns (no ROS needed)."""
    # Embedded list matching GraspLibrary::initBuiltinPatterns()
    patterns = [
        ("open",              "All fingers fully extended"),
        ("close",             "Full fist — thumb wrapped over index"),
        ("home",              "Neutral semi-grasp (default rest pose)"),
        ("pinch",             "Precision pinch — thumb + index tip-to-tip"),
        ("tripod",            "Tripod — thumb + index + middle"),
        ("power_grasp",       "Power grasp — thumb wrapped over fingers"),
        ("hook_grasp",        "Hook grip — PIP+DIP flexed for pulling/carrying"),
        ("spherical_grasp",   "Spherical grasp — fingers spread for round objects"),
        ("cylindrical_grasp", "Cylindrical grasp — uniform curl for bottles/cups"),
        ("lateral_pinch",     "Lateral (key) pinch — thumb against index side"),
        ("three_finger",      "Three-finger precision — thumb + index + middle"),
        ("point",             "Pointing — index extended, others closed"),
        ("thumbs_up",         "Thumbs up — thumb extended, fingers closed"),
        ("peace_sign",        "Peace / V sign — index+middle extended"),
        ("trigger",           "Trigger finger — index extended for pressing buttons"),
    ]

    print(f"\n{Colors.BOLD}Available Grasp Patterns ({len(patterns)}){Colors.RESET}")
    print("=" * 60)
    for name, desc in patterns:
        print(f"  {Colors.CYAN}{name:<22}{Colors.RESET} {desc}")
    print()

    print(f"{Colors.BOLD}CLI Commands{Colors.RESET}")
    print("=" * 60)
    print(f"  {Colors.GREEN}reach <x> <y> <z>{Colors.RESET} [--ori qx qy qz qw] [--timeout 5.0]")
    print(f"  {Colors.GREEN}grasp <pattern>{Colors.RESET} [--duration 0.5]")
    print(f"  {Colors.GREEN}reach_and_grasp <x> <y> <z> <pattern>{Colors.RESET} [--ori ...] [--duration 0.5]")
    print(f"  {Colors.GREEN}interactive{Colors.RESET}  — 交互模式（3-DOF / 6-DOF RPY）")
    print(f"  {Colors.GREEN}home{Colors.RESET}")
    print(f"  {Colors.GREEN}open{Colors.RESET}")
    print(f"  {Colors.GREEN}list{Colors.RESET}")
    print()


# ===========================================================================
# Interactive mode
# ===========================================================================
def cmd_interactive(args):
    """Interactive REPL with persistent ROS connection for low-latency control.

    Formats:
      <x> <y> <z> <pattern>           → 3-DOF arm + grasp
      <x> <y> <z> <yaw> <pattern>     → position + yaw (rad, relative to palm)
      <x> <y> <z> rpy R P Y <p> [dur] → 6-DOF RPY (relative to palm)
      grasp <pattern> [dur]            → hand only
      home | open | reach <x> <y> <z> → simple commands

    RPY and yaw are RELATIVE to current palm orientation.
    RPY(0,0,0) = hold current. Wrist authority is ±2.0 rad.
    """
    rclpy.init(args=[])
    node = Node('action_cli_interactive')
    pub = node.create_publisher(ActionCommand, '/action_command', 10)

    # Give ROS discovery a moment
    time.sleep(0.3)

    print(f"""
{Colors.BOLD}{Colors.CYAN}╔══════════════════════════════════════════════════════╗
║   G1 Right Arm + Wuji Hand 2 — Interactive CLI     ║
║   3-DOF: x y z pattern                               ║
║   6-DOF: x y z rpy R P Y pattern [dur]              ║
║   RPY/yaw are RELATIVE to current palm orientation   ║
╚══════════════════════════════════════════════════════╝{Colors.RESET}

{Colors.GREEN}Examples:{Colors.RESET}
  {Colors.CYAN}0.20 -0.05 -0.10 close{Colors.RESET}              → 3-DOF reach + grasp
  {Colors.CYAN}0.20 -0.05 -0.10 rpy 0 0 0 close{Colors.RESET}    → RPY=0 (hold palm ori)
  {Colors.CYAN}0.20 -0.05 -0.10 rpy 0 0 1.57 pinch{Colors.RESET} → yaw=+90°, pinch
  {Colors.CYAN}0.20 -0.05 -0.10 rpy 0 1.57 0 close{Colors.RESET} → pitch=+90°, close
  {Colors.CYAN}home{Colors.RESET}                                  → ready pose + home grasp
  {Colors.CYAN}open{Colors.RESET}                                  → open hand

{Colors.BOLD}Grasp patterns:{Colors.RESET} open, close, home, pinch, tripod, three_finger,
              power_grasp, cylindrical_grasp, spherical_grasp,
              hook_grasp, lateral_pinch, point, thumbs_up,
              peace_sign, trigger
""")

    try:
        while rclpy.ok():
            line = input(f"{Colors.BOLD}> {Colors.RESET}").strip()
            if not line:
                continue

            parts = line.split()
            cmd_first = parts[0].lower()

            # ── quit ──
            if cmd_first in ('quit', 'exit', 'q'):
                info("Goodbye!")
                break

            # ── help ──
            if cmd_first in ('help', '?'):
                print(f"""\n{Colors.CYAN}Format:{Colors.RESET}
  {Colors.GREEN}<x> <y> <z> <pattern>{Colors.RESET}               → 3-DOF arm + grasp
  {Colors.GREEN}<x> <y> <z> rpy R P Y <p>{Colors.RESET}         → 6-DOF (RPY relative to palm)
  {Colors.GREEN}<x> <y> <z> rpy R P Y <p> <dur>{Colors.RESET}   → 6-DOF + duration

{Colors.CYAN}Notes:{Colors.RESET}
  RPY(0,0,0) = hold current palm orientation. Wrist authority: ±2.0 rad.

{Colors.CYAN}Examples:{Colors.RESET}
  {Colors.GREEN}0.20 -0.05 -0.10 close{Colors.RESET}              → 3-DOF grasp
  {Colors.GREEN}0.20 -0.05 -0.10 rpy 0 0 0 close{Colors.RESET}    → 6-DOF, identity rotation
  {Colors.GREEN}0.20 -0.05 -0.10 rpy 0 1.57 0 pinch{Colors.RESET} → pitch=90°, pinch
  {Colors.GREEN}home | open | list{Colors.RESET}
""")
                continue

            # ── list ──
            if cmd_first == 'list':
                cmd_list(None)
                continue

            # ── home ──
            if cmd_first == 'home':
                cmd = ActionCommand()
                cmd.action_type = 'home'
                pub.publish(cmd)
                ok("HOME → ready pose + home grasp [3-DOF]")
                continue

            # ── open ──
            if cmd_first == 'open':
                cmd = ActionCommand()
                cmd.action_type = 'grasp'
                cmd.grasp_type = 'open'
                cmd.grasp_duration = 0.15
                pub.publish(cmd)
                ok("OPEN hand")
                continue

            # ── grasp <pattern> [duration] ──
            if cmd_first == 'grasp' and len(parts) >= 2:
                cmd = ActionCommand()
                cmd.action_type = 'grasp'
                cmd.grasp_type = parts[1]
                cmd.grasp_duration = float(parts[2]) if len(parts) >= 3 else 0.3
                pub.publish(cmd)
                ok(f"GRASP → {parts[1]}  (duration={cmd.grasp_duration}s)")
                continue

            # ── reach <x> <y> <z> ──
            if cmd_first == 'reach' and len(parts) >= 4:
                cmd = ActionCommand()
                cmd.action_type = 'reach'
                cmd.target_x = float(parts[1])
                cmd.target_y = float(parts[2])
                cmd.target_z = float(parts[3])
                cmd.ori_w = 1.0
                cmd.convergence_timeout = float(parts[4]) if len(parts) >= 5 else 5.0
                pub.publish(cmd)
                ok(f"REACH → [{cmd.target_x:.2f}, {cmd.target_y:.2f}, {cmd.target_z:.2f}] [3-DOF]")
                continue

            # ── <x> <y> <z> [rpy R P Y] <pattern> [duration] ──
            if len(parts) >= 4:
                import math

                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                except ValueError:
                    warn(f"Cannot parse position: '{parts[0]} {parts[1]} {parts[2]}'")
                    continue

                # ── RPY mode: x y z rpy R P Y pattern [dur] ──
                if parts[3].lower() == 'rpy' and len(parts) >= 8:
                    try:
                        roll  = float(parts[4])
                        pitch = float(parts[5])
                        yaw_  = float(parts[6])
                    except ValueError:
                        warn(f"Cannot parse RPY: '{parts[4]} {parts[5]} {parts[6]}'")
                        continue

                    pattern = parts[7]
                    duration = 0.15
                    if len(parts) >= 9:
                        try:
                            duration = float(parts[8])
                        except ValueError:
                            pass

                    # RPY (ZYX) → quaternion (relative to current palm orientation)
                    # The hand_controller composes this with the current palm ori:
                    #   target_abs = current_hand_ori * q_user
                    # RPY(0,0,0) → identity → HOLD current orientation
                    # RPY(0,0,0.5) → yaw 0.5 rad relative to palm
                    cr = math.cos(roll / 2);  sr = math.sin(roll / 2)
                    cp = math.cos(pitch / 2); sp = math.sin(pitch / 2)
                    cy = math.cos(yaw_ / 2);  sy = math.sin(yaw_ / 2)

                    qw = cr*cp*cy + sr*sp*sy
                    qx = sr*cp*cy - cr*sp*sy
                    qy = cr*sp*cy + sr*cp*sy
                    qz = cr*cp*sy - sr*sp*cy

                    cmd = ActionCommand()
                    cmd.action_type = 'reach_and_grasp'
                    cmd.target_x = x; cmd.target_y = y; cmd.target_z = z
                    cmd.ori_x = float(qx); cmd.ori_y = float(qy)
                    cmd.ori_z = float(qz); cmd.ori_w = float(qw)
                    cmd.grasp_type = pattern
                    cmd.grasp_duration = duration
                    cmd.convergence_timeout = 3.0
                    cmd.settle_time = 0.03

                    pub.publish(cmd)
                    ok(f"→ [{x:.2f}, {y:.2f}, {z:.2f}] "
                       f"rpy=[{roll:.2f},{pitch:.2f},{yaw_:.2f}] → {pattern} ({duration}s) [6-DOF]")
                    continue

                # ── Detect yaw vs pattern in parts[3] ──
                is_yaw = False
                yaw_val = 0.0
                try:
                    yaw_val = float(parts[3])
                    is_yaw = True
                except ValueError:
                    pass

                if is_yaw and len(parts) >= 5:
                    # Format: x y z YAW pattern [duration]
                    yaw_ = yaw_val
                    pattern = parts[4]
                    duration = 0.15
                    if len(parts) >= 6:
                        try:
                            duration = float(parts[5])
                        except ValueError:
                            pass

                    cy = math.cos(yaw_ / 2.0)
                    sy = math.sin(yaw_ / 2.0)

                    # Pure yaw quat (relative to current palm orientation)
                    # hand_controller composes: target_abs = current_hand_ori * q_yaw
                    qw, qx, qy, qz = cy, 0.0, 0.0, sy

                    cmd = ActionCommand()
                    cmd.action_type = 'reach_and_grasp'
                    cmd.target_x = x; cmd.target_y = y; cmd.target_z = z
                    cmd.ori_x = float(qx); cmd.ori_y = float(qy)
                    cmd.ori_z = float(qz); cmd.ori_w = float(qw)
                    cmd.grasp_type = pattern
                    cmd.grasp_duration = duration
                    cmd.convergence_timeout = 3.0
                    cmd.settle_time = 0.03

                    pub.publish(cmd)
                    ok(f"→ [{x:.2f}, {y:.2f}, {z:.2f}] yaw={yaw_:.2f}rad → {pattern} ({duration}s)")
                    continue

                elif not is_yaw:
                    # Format: x y z PATTERN [duration] (3-DOF)
                    pattern = parts[3]
                    duration = 0.15
                    if len(parts) >= 5:
                        try:
                            duration = float(parts[4])
                        except ValueError:
                            pass

                    cmd = ActionCommand()
                    cmd.action_type = 'reach_and_grasp'
                    cmd.target_x = x; cmd.target_y = y; cmd.target_z = z
                    cmd.ori_w = 1.0
                    cmd.grasp_type = pattern
                    cmd.grasp_duration = duration
                    cmd.convergence_timeout = 3.0
                    cmd.settle_time = 0.03

                    pub.publish(cmd)
                    ok(f"→ [{x:.2f}, {y:.2f}, {z:.2f}] → {pattern} ({duration}s) [3-DOF]")
                    continue

                else:
                    warn(f"Missing grasp pattern. Format: x y z [rpy R P Y] <pattern>")
                    continue

            # ── unrecognized ──
            warn(f"无法解析: '{line}'. 输入 'help' 查看帮助。")

    except (KeyboardInterrupt, EOFError):
        print()
        info("Goodbye!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ===========================================================================
# Main — argparse
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(
        description='G1 Right Arm + Wuji Hand 2 — Action CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 action_cli.py reach 0.20 -0.05 -0.10
  python3 action_cli.py reach 0.20 -0.05 -0.10 --ori 0 0 0 1
  python3 action_cli.py grasp pinch
  python3 action_cli.py reach_and_grasp 0.20 -0.05 -0.10 pinch
  python3 action_cli.py open
  python3 action_cli.py home
  python3 action_cli.py list
        """)

    subparsers = parser.add_subparsers(dest='command', help='Action command')

    # ── reach ──
    p_reach = subparsers.add_parser('reach', help='Move arm to target position')
    p_reach.add_argument('x', type=float, help='X (forward, m)')
    p_reach.add_argument('y', type=float, help='Y (left, m)')
    p_reach.add_argument('z', type=float, help='Z (up, m)')
    p_reach.add_argument('--ori', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'),
                         help='Target orientation quaternion (default: 0 0 0 1)')
    p_reach.add_argument('--timeout', type=float, default=5.0, help='Convergence timeout (s)')
    p_reach.add_argument('--settle', type=float, default=0.15, help='Settle time after convergence (s)')

    # ── grasp ──
    p_grasp = subparsers.add_parser('grasp', help='Execute grasp pattern')
    p_grasp.add_argument('pattern', type=str, help='Grasp pattern name (open/pinch/...)')
    p_grasp.add_argument('--duration', type=float, default=0.08, help='Interpolation duration (s)')

    # ── reach_and_grasp ──
    p_rng = subparsers.add_parser('reach_and_grasp', help='Reach to target, then grasp')
    p_rng.add_argument('x', type=float, help='X (forward, m)')
    p_rng.add_argument('y', type=float, help='Y (left, m)')
    p_rng.add_argument('z', type=float, help='Z (up, m)')
    p_rng.add_argument('pattern', type=str, help='Grasp pattern name')
    p_rng.add_argument('--ori', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'),
                       help='Target orientation quaternion')
    p_rng.add_argument('--duration', type=float, default=0.08, help='Grasp interpolation duration (s)')
    p_rng.add_argument('--timeout', type=float, default=5.0, help='Convergence timeout (s)')
    p_rng.add_argument('--settle', type=float, default=0.15, help='Settle time after convergence (s)')

    # ── shortcuts ──
    subparsers.add_parser('home', help='Go to ready pose + home grasp')
    subparsers.add_parser('open', help='Open hand (all fingers extended)')
    subparsers.add_parser('list', help='List available grasp patterns and commands')
    subparsers.add_parser('interactive', help='Interactive REPL — persistent connection, low latency')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    # Dispatch
    dispatch = {
        'reach': cmd_reach,
        'grasp': cmd_grasp,
        'reach_and_grasp': cmd_reach_and_grasp,
        'home': cmd_home,
        'open': cmd_open,
        'list': cmd_list,
        'interactive': cmd_interactive,
    }

    dispatch[args.command](args)


if __name__ == '__main__':
    main()
