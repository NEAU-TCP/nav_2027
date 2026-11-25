# Copyright 2026
# Licensed under the Apache License, Version 2.0.

import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def test_gpu_snapshot_keeps_one_acquisition_time(tmp_path, monkeypatch):
    # Keep the fixture separate from any running navigation stack.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(180 + os.getpid() % 40))
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    rclpy.init(args=[])
    node = rclpy.create_node("snapshot_converter_test")
    publisher = node.create_publisher(PointCloud2, "/snapshot_test/raw", 10)
    received = []
    subscription = node.create_subscription(
        PointCloud2, "/snapshot_test/velodyne_points", received.append,
        qos_profile_sensor_data,
    )
    executable = Path(get_package_prefix("ign_sim_pointcloud_tool")) / (
        "lib/ign_sim_pointcloud_tool/ign_sim_pointcloud_tool_node"
    )
    header = Header(frame_id="front_mid360")
    header.stamp.sec = 12
    header.stamp.nanosec = 345000000
    # More than one old artificial scan period; all points are in the vertical FOV.
    points = np.asarray([(2.0 + i * 0.001, 0.0, 0.1) for i in range(4000)], dtype=np.float32)
    cloud = point_cloud2.create_cloud_xyz32(header, points)
    process = None
    try:
        with (tmp_path / "converter.log").open("w") as output:
            process = subprocess.Popen(
                [str(executable), "--ros-args", "-r", "__ns:=/snapshot_test",
                 "-p", "pcd_topic:=raw"], stdout=output, stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 10.0
            next_send = 0.0
            while time.monotonic() < deadline and not received:
                assert process.poll() is None, (tmp_path / "converter.log").read_text()
                if time.monotonic() >= next_send:
                    publisher.publish(cloud)
                    next_send = time.monotonic() + 0.1
                rclpy.spin_once(node, timeout_sec=0.02)
            assert received, "Converted point cloud did not arrive"
            result = received[-1]
            assert result.header == header
            output_points = list(point_cloud2.read_points(
                result, field_names=("x", "y", "z", "time"), skip_nans=True,
            ))
            assert len(output_points) == len(points)
            for expected, actual in zip(points, output_points):
                assert tuple(actual[i] for i in range(3)) == tuple(expected)
                assert actual[3] == 0.0, "Snapshot points must not acquire invented scan offsets"
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
