#!/usr/bin/env python

import rospy
from view_controller_msgs.msg import CameraPlacement
from ds4_driver.msg import Status

from ds4_utils.trackpad_handler import TrackpadHandler

import math


class ControllerNode(object):
    def __init__(self):
        self._target_frame = rospy.get_param('~target_frame', 'world')
        tap_timeout = rospy.get_param('~tap_timeout', 0.5)
        self._altitude = rospy.get_param('~initial_altitude', 0.4)
        self._altitude_scale = rospy.get_param('~altitude_scale', 1)
        self._azimuth = rospy.get_param('~initial_azimuth', 0.4)
        self._azimuth_scale = rospy.get_param('~azimuth_scale', -2)
        self._distance = rospy.get_param('~initial_distance', 10)
        self._distance_scale = rospy.get_param('~distance_scale', 8)
        rate = rospy.get_param('~rate', 10)
        self._min_interval = rospy.Duration.from_sec(1.0 / rate)
        self._last_pub_time = rospy.Time(0)
        topic_name = rospy.get_param('~topic_name', '/rviz/camera_placement')

        self._touchpad = TrackpadHandler(tap_timeout=tap_timeout)

        self._placement_msg = CameraPlacement()
        self._placement_msg.interpolation_mode = CameraPlacement.SPHERICAL
        self._placement_msg.time_from_start = self._min_interval
        self._placement_msg.target_frame = self._target_frame
        self._placement_msg.eye.header.frame_id = self._target_frame
        self._placement_msg.focus.header.frame_id = self._target_frame
        self._placement_msg.focus.point.x = 0
        self._placement_msg.focus.point.y = 0
        self._placement_msg.focus.point.z = 0
        self._placement_msg.up.header.frame_id = self._target_frame
        self._placement_msg.up.vector.x = 0
        self._placement_msg.up.vector.y = 0
        self._placement_msg.up.vector.z = 1.0

        self._pub = rospy.Publisher(topic_name, CameraPlacement, queue_size=50, latch=True)
        self._sub = rospy.Subscriber('status', Status, self.cb_status, queue_size=1)

        self.publish_current_placement()

    def cb_status(self, msg):
        """
        :type msg: Status
        :return:
        """
        now = rospy.Time.now()
        if now - self._last_pub_time < self._min_interval:
            return

        state, offset = self._touchpad.process(msg.touch0, msg.touch1)

        if state is TrackpadHandler.STATE_ONE_FINGER_DRAG:
            self._azimuth += offset[0] * self._azimuth_scale
            self._altitude += offset[1] * self._altitude_scale
        elif state is TrackpadHandler.STATE_TWO_FINGER_DRAG:
            self._distance += offset[1] * self._distance_scale
        else:
            return

        self.publish_current_placement()
        self._last_pub_time = now

    def publish_current_placement(self):
        self._distance = max(0, self._distance)
        self._altitude = min(math.pi / 2, self._altitude)
        self._altitude = max(-math.pi / 2, self._altitude)

        # Convert altitude, azimuth, distance to xyz
        x, y, z = ControllerNode.to_cartesian(self._distance, self._altitude, self._azimuth)
        self._placement_msg.eye.point.x = x
        self._placement_msg.eye.point.y = y
        self._placement_msg.eye.point.z = z

        self._pub.publish(self._placement_msg)

    @staticmethod
    def to_cartesian(distance, altitude, azimuth):
        x = distance * math.cos(altitude) * math.cos(azimuth)
        y = distance * math.cos(altitude) * math.sin(azimuth)
        z = distance * math.sin(altitude)

        return x, y, z


def main():
    rospy.init_node('view_control')

    ControllerNode()

    rospy.spin()


if __name__ == '__main__':
    main()
