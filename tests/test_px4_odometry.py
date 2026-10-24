#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-only
"""Exercise the adapter on isolated simulation topics; never publish to PX4."""
import argparse
import copy
import json
import math
import subprocess
import time



def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleOdometry
    from rclpy.qos import qos_profile_sensor_data

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('odometry_conversion_acceptance')
    received = []
    output = '/simulation/vio/adapter_output'
    source = '/simulation/vio/adapter_input'
    subscription = node.create_subscription(VehicleOdometry, output, received.append,
                                            qos_profile_sensor_data)
    publisher = node.create_publisher(Odometry, source, qos_profile_sensor_data)
    process = subprocess.Popen([args.binary, '--ros-args', '-r', '__node:=adapter_acceptance',
                                '-p', 'input_topic:='+source, '-p', 'output_topic:='+output])

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def measurement():
        msg = Odometry()
        msg.header.frame_id = 'global'
        msg.child_frame_id = 'imu'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.
        msg.pose.pose.position.x = 1.
        msg.pose.pose.position.y = 2.
        msg.pose.pose.position.z = 3.
        msg.twist.twist.linear.x = .5
        msg.twist.twist.linear.y = .2
        msg.twist.twist.linear.z = .1
        for i in range(6):
            msg.pose.covariance[i*7] = .1*(i+1)
            msg.twist.covariance[i*7] = .2*(i+1)
        return msg

    try:
        spin(3)
        assert publisher.get_subscription_count() == 1
        msg = measurement()
        publisher.publish(msg)
        spin(.3)
        assert len(received) == 1, len(received)
        result = received[-1]
        assert list(result.position) == [2., 1., -3.]
        assert result.pose_frame == VehicleOdometry.POSE_FRAME_FRD
        assert result.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        assert all(abs(a-b) < 1e-6 for a,b in zip(result.velocity,[.5,-.2,-.1]))
        assert all(abs(a-b) < 1e-6 for a,b in zip(result.position_variance,[.2,.1,.3]))
        assert abs(sum(v*v for v in result.q)-1) < 1e-6
        assert result.timestamp_sample == msg.header.stamp.sec*1000000+msg.header.stamp.nanosec//1000
        invalid = []
        for kind in ('wrong_frame','zero_quaternion','nan_position','negative_covariance',
                     'asymmetric_covariance','stale','future','overspeed','excessive_angular_rate'):
            bad = measurement()
            if kind == 'wrong_frame': bad.header.frame_id = 'unknown'
            if kind == 'zero_quaternion': bad.pose.pose.orientation.w = 0.
            if kind == 'nan_position': bad.pose.pose.position.x = math.nan
            if kind == 'negative_covariance': bad.pose.covariance[0] = -1.
            if kind == 'asymmetric_covariance': bad.pose.covariance[1] = .05
            if kind == 'stale': bad.header.stamp.sec -= 1
            if kind == 'future': bad.header.stamp.sec += 1
            if kind == 'overspeed': bad.twist.twist.linear.x = 20.
            if kind == 'excessive_angular_rate': bad.twist.twist.angular.x = 100.
            publisher.publish(bad)
            spin(.1)
            assert len(received) == 1, kind
            invalid.append(kind)
        duplicate = copy.deepcopy(msg)
        duplicate.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(duplicate)
        spin(.1)
        assert len(received) == 2
        publisher.publish(duplicate)
        spin(.05)
        assert len(received) == 2, 'duplicate accepted'
        spin(.7)
        assert len(received) == 2, 'stale data resent'
        print(json.dumps({'valid_conversion':True,'rejected':invalid+['duplicate'],
                          'stale_data_not_resent':True,'output_topic':output}),flush=True)
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
