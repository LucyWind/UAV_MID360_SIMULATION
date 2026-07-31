#!/usr/bin/env python3
"""Real-time plot and CSV recorder for nav_msgs/Odometry position.z."""

import argparse
import csv
from collections import deque
from datetime import datetime
from pathlib import Path
import threading
import tkinter as tk

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class OdomZPlotter(Node):
    def __init__(self, topic: str, window_seconds: float, csv_path: Path) -> None:
        super().__init__("odom_z_plotter")
        self.window_seconds = window_seconds
        self.samples = deque()
        self.start_time = None
        self.last_z = None
        self.lock = threading.Lock()
        self.closed = False
        self.last_csv_flush_time = 0.0

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(("time_s", "z_m"))

        odom_qos = QoSProfile(
            depth=500,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Odometry, topic, self.odom_callback, odom_qos
        )
        self.get_logger().info(f"Plotting {topic}.pose.pose.position.z")
        self.get_logger().info(f"Recording CSV to {csv_path}")

    def odom_callback(self, msg: Odometry) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time
        z = msg.pose.pose.position.z
        with self.lock:
            self.samples.append((elapsed, z))
            self.last_z = z
            oldest_allowed = elapsed - self.window_seconds
            while self.samples and self.samples[0][0] < oldest_allowed:
                self.samples.popleft()

            self.csv_writer.writerow((f"{elapsed:.6f}", f"{z:.9f}"))
            if elapsed - self.last_csv_flush_time >= 1.0:
                self.csv_file.flush()
                self.last_csv_flush_time = elapsed

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.csv_file.flush()
        self.csv_file.close()
        self.destroy_node()


class CsvZData:
    def __init__(self, csv_path: Path) -> None:
        with csv_path.open(newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames is None or not {"time_s", "z_m"}.issubset(
                reader.fieldnames
            ):
                raise ValueError("CSV must contain time_s and z_m columns")
            self.samples = deque(
                (float(row["time_s"]), float(row["z_m"])) for row in reader
            )

        if not self.samples:
            raise ValueError("CSV contains no data rows")

        first_time = self.samples[0][0]
        if first_time != 0.0:
            self.samples = deque(
                (timestamp - first_time, z) for timestamp, z in self.samples
            )
        self.window_seconds = max(self.samples[-1][0], 1.0)
        self.last_z = self.samples[-1][1]

    def close(self) -> None:
        pass


class PlotWindow:
    MARGIN_LEFT = 72
    MARGIN_RIGHT = 24
    MARGIN_TOP = 34
    MARGIN_BOTTOM = 52

    def __init__(self, node, title: str, live: bool = True) -> None:
        self.node = node
        self.live = live
        self.root = tk.Tk()
        self.root.title(f"Odometry Z — {title}")
        self.root.geometry("1000x600")
        self.canvas = tk.Canvas(self.root, bg="#111827", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)
        self.running = True
        self.spin_thread = None
        if self.live:
            self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.spin_thread.start()
            self.root.after(50, self.update)
        else:
            self.canvas.bind("<Configure>", lambda _event: self.draw())
            self.root.after_idle(self.draw)

    def spin_ros(self) -> None:
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def shutdown(self) -> None:
        if not self.running:
            return
        self.running = False
        if self.live and rclpy.ok():
            rclpy.shutdown()
        if self.spin_thread is not None:
            self.spin_thread.join(timeout=1.0)
        self.node.close()
        self.root.destroy()

    def update(self) -> None:
        if not self.running or (self.live and not rclpy.ok()):
            return
        self.draw()
        if self.live:
            self.root.after(50, self.update)

    def draw(self) -> None:
        width = max(self.canvas.winfo_width(), 200)
        height = max(self.canvas.winfo_height(), 160)
        left = self.MARGIN_LEFT
        right = width - self.MARGIN_RIGHT
        top = self.MARGIN_TOP
        bottom = height - self.MARGIN_BOTTOM

        self.canvas.delete("all")
        self.canvas.create_text(
            width / 2,
            16,
            text="里程计 Z 轴高度",
            fill="#f9fafb",
            font=("Sans", 13, "bold"),
        )
        self.canvas.create_rectangle(left, top, right, bottom, outline="#6b7280")

        lock = getattr(self.node, "lock", None)
        if lock is None:
            samples = list(self.node.samples)
            last_z = self.node.last_z
        else:
            with lock:
                samples = list(self.node.samples)
                last_z = self.node.last_z
        if not samples:
            self.canvas.create_text(
                width / 2,
                height / 2,
                text="等待 /Odometry 数据……",
                fill="#d1d5db",
                font=("Sans", 14),
            )
            return

        t_max = samples[-1][0]
        t_min = max(0.0, t_max - self.node.window_seconds)
        if t_max <= t_min:
            t_max = t_min + 1.0

        z_values = [sample[1] for sample in samples]
        z_min = min(z_values)
        z_max = max(z_values)
        padding = max((z_max - z_min) * 0.12, 0.05)
        z_min -= padding
        z_max += padding

        for index in range(6):
            ratio = index / 5
            y = bottom - ratio * (bottom - top)
            z_tick = z_min + ratio * (z_max - z_min)
            self.canvas.create_line(left, y, right, y, fill="#374151")
            self.canvas.create_text(
                left - 8, y, text=f"{z_tick:.2f}", fill="#d1d5db", anchor="e"
            )

        for index in range(6):
            ratio = index / 5
            x = left + ratio * (right - left)
            t_tick = t_min + ratio * (t_max - t_min)
            self.canvas.create_line(x, top, x, bottom, fill="#273244")
            self.canvas.create_text(
                x, bottom + 17, text=f"{t_tick:.1f}", fill="#d1d5db"
            )

        self.canvas.create_text(
            (left + right) / 2,
            height - 14,
            text="时间 (s)",
            fill="#e5e7eb",
        )
        self.canvas.create_text(
            17,
            (top + bottom) / 2,
            text="Z (m)",
            fill="#e5e7eb",
            angle=90,
        )

        points = []
        for timestamp, z in samples:
            x = left + (timestamp - t_min) / (t_max - t_min) * (right - left)
            y = bottom - (z - z_min) / (z_max - z_min) * (bottom - top)
            points.extend((x, y))
        if len(points) >= 4:
            self.canvas.create_line(*points, fill="#38bdf8", width=2)

        self.canvas.create_text(
            right - 8,
            top + 14,
            text=f"当前 Z: {last_z:.3f} m",
            fill="#fbbf24",
            anchor="e",
            font=("Sans", 11, "bold"),
        )

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    workspace = Path(__file__).resolve().parent.parent
    default_csv = workspace / "logs" / (
        f"odom_z_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument(
        "--window", type=float, default=30.0, help="Visible time window in seconds"
    )
    parser.add_argument("--csv", type=Path, default=default_csv)
    parser.add_argument(
        "--input-csv",
        type=Path,
        help="Open an existing time_s,z_m CSV instead of subscribing to ROS",
    )
    args, ros_args = parser.parse_known_args()

    if args.window <= 0.0:
        parser.error("--window must be greater than zero")

    if args.input_csv is not None:
        try:
            data = CsvZData(args.input_csv.expanduser().resolve())
        except (OSError, ValueError) as error:
            parser.error(str(error))
        PlotWindow(data, args.input_csv.name, live=False).run()
        return

    rclpy.init(args=ros_args)
    node = OdomZPlotter(args.topic, args.window, args.csv.resolve())
    window = PlotWindow(node, args.topic, live=True)
    try:
        window.run()
    finally:
        if rclpy.ok():
            node.close()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
