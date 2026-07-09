#!/usr/bin/env python3
"""
Record /debug/position_error and plot time-series curve.

Usage:
    source ~/g1_ws/install/setup.bash
    python3 scripts/plot_error.py                    # auto-named with timestamp
    python3 scripts/plot_error.py --duration 30      # record 30 seconds
    python3 scripts/plot_error.py -o my_plot.png     # custom output file

This is a diagnostic utility for the G1 right arm reaching demo. It subscribes
to the /debug/position_error topic published by reach_node, records the data,
prints summary statistics, saves a CSV, and generates a matplotlib plot.
"""
import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


# ============================================================================
# MODULE: ErrorRecorder — ROS2 node that subscribes to /debug/position_error
# ============================================================================
class ErrorRecorder(Node):
    """Subscribe to /debug/position_error and record (elapsed, error) pairs.

    The recorded data is later consumed by main() for summary statistics,
    CSV export, and matplotlib plotting.

    Attributes:
        data: List of (elapsed_seconds, error_meters) tuples.
        t0: Wall-clock timestamp when recording started.
        duration: If set, automatically stops recording after this many seconds.
    """
    def __init__(self, duration=None):
        super().__init__('error_recorder')
        self.sub = self.create_subscription(
            Float64, '/debug/position_error', self.cb, 10)
        self.data = []  # list of (elapsed_sec, error_m)
        self.t0 = time.time()
        self.duration = duration
        self.get_logger().info('Recording error data... (Ctrl+C to stop)')

    def cb(self, msg):
        """Callback for each /debug/position_error message.

        Args:
            msg: std_msgs.msg.Float64 containing the current position error in meters.
        """
        elapsed = time.time() - self.t0
        self.data.append((elapsed, msg.data))
        if self.duration and elapsed >= self.duration:
            self.get_logger().info(f'Duration {self.duration}s reached.')
            raise KeyboardInterrupt


# ============================================================================
# MODULE: main — parse args, spin recorder, print summary, export CSV, plot
# ============================================================================
def main():
    """Entry point: record position error, then summarise, export, and plot."""
    # --- Argument parsing ---
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output', default='',
                        help='Output plot filename (default: error_plot_<timestamp>.png)')
    parser.add_argument('-d', '--duration', type=float, default=0,
                        help='Record duration in seconds (0=until Ctrl+C)')
    parser.add_argument('--csv', default='', help='Also save raw CSV')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting, just print summary')
    args = parser.parse_args()

    # --- Data collection: spin the ROS2 recorder node ---
    rclpy.init()
    node = ErrorRecorder(duration=args.duration or None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # already shut down by spin()

    if not node.data:
        print("No data recorded. Did you send a target point?")
        return

    # Convert to plottable arrays (meters → centimeters)
    times = [d[0] for d in node.data]
    errors = [d[1] * 100 for d in node.data]  # m → cm

    # --- Summary statistics ---
    print(f"\n--- Error summary ---")
    print(f"  Samples:  {len(node.data)}")
    print(f"  Duration: {times[-1]:.1f}s")
    print(f"  Start:    {errors[0]:.1f} cm")
    print(f"  Final:    {errors[-1]:.1f} cm")
    print(f"  Min:      {min(errors):.1f} cm")

    # --- Determine output paths (auto-generate with timestamp if not given) ---
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    workspace_dir = Path.home() / 'g1_ws'

    if args.output:
        out_path = Path(args.output)
    else:
        out_path = workspace_dir / f'error_plot_{timestamp}.png'

    if args.csv:
        csv_path = Path(args.csv)
    else:
        csv_path = workspace_dir / f'error_data_{timestamp}.csv'

    # --- CSV export ---
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time_s', 'error_cm'])
        for t, e in zip(times, errors):
            w.writerow([f'{t:.3f}', f'{e:.2f}'])
    print(f"  CSV saved: {csv_path.resolve()}")

    # --- Matplotlib plotting ---
    if args.no_plot:
        print("  Plot skipped (--no-plot).")
        return

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[!] matplotlib not installed. Install with:")
        print("    pip install matplotlib")
        print(f"\nRaw data (time_s, error_cm):")
        for t, e in zip(times[::max(1, len(times)//20)], errors[::max(1, len(errors)//20)]):
            print(f"  {t:6.1f}s  {e:6.1f} cm")
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, errors, 'b-', linewidth=1.5, label='Position Error')
    # Horizontal reference lines for key error thresholds
    ax.axhline(y=1.0, color='g', linestyle='--', linewidth=1, label='Target (1 cm)')
    ax.axhline(y=0.5, color='r', linestyle=':', linewidth=1, label='Converged (0.5 cm)')

    # Annotate start and end points on the curve
    ax.annotate(f'{errors[0]:.1f} cm', xy=(times[0], errors[0]),
                xytext=(times[0]+1, errors[0]+2),
                arrowprops=dict(arrowstyle='->', color='gray'), fontsize=9)
    ax.annotate(f'{errors[-1]:.1f} cm', xy=(times[-1], errors[-1]),
                xytext=(times[-1]-5, errors[-1]+3),
                arrowprops=dict(arrowstyle='->', color='gray'), fontsize=9)

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Position Error (cm)')
    ax.set_title('G1 Right Arm — Position Error Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"  Plot saved: {out_path.resolve()}")
    plt.close()


if __name__ == '__main__':
    main()
