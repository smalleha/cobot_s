#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Python ROS node to subscribe AR markers, compute averaged positions,
provide processed data via a service.
"""
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, Point
from tf2_ros import TransformBroadcaster
from custom_msgs.srv import QrData, QrDataResponse  # Replace with your package and srv name

# Marker categories
OBJECT, PICK, PLACE = 0, 1, 2

class QrServiceNode:
    def __init__(self):
        rospy.init_node('qr_service_node')

        # storage for temporary accumulation and final positions
        self.temp = {k: Point() for k in (OBJECT, PICK, PLACE)}
        self.seq_count = {k: 0 for k in (OBJECT, PICK, PLACE)}
        self.real_position = {k: Point() for k in (OBJECT, PICK, PLACE)}
        self.is_mark_empty = {k: True for k in (OBJECT, PICK, PLACE)}

        # TF broadcaster
        self.broadcaster = TransformBroadcaster()

        # Service to provide processed QR data
        self.srv = rospy.Service('get_qr_data', QrData, self.handle_qr_service)
        rospy.loginfo("Service 'get_qr_data' ready.")

        # Subscriber for AR markers
        topic = rospy.get_param('~topic', '/ar_pose_marker')
        rospy.Subscriber(topic, AlvarMarkers, self.marker_callback)
        rospy.loginfo(f"Subscribed to {topic}")

        rospy.spin()

    def marker_callback(self, msg):
        # reset state
        for k in self.is_mark_empty:
            self.is_mark_empty[k] = True

        if not msg.markers:
            # clear positions if no markers
            for k in self.real_position:
                self.real_position[k] = Point()
            rospy.loginfo("no data!")
            return

        # accumulate and compute averages
        for marker in msg.markers:
            id_num = marker.id
            if id_num in (OBJECT, PICK, PLACE):
                p = marker.pose.pose.position
                self.temp[id_num].x += p.x
                self.temp[id_num].y += p.y
                self.temp[id_num].z += p.z

                self.seq_count[id_num] += 1
                if self.seq_count[id_num] == 3:
                    # average over 3 samples
                    self.real_position[id_num].x = self.temp[id_num].x / 3.0
                    self.real_position[id_num].y = self.temp[id_num].y / 3.0
                    self.real_position[id_num].z = self.temp[id_num].z / 3.0

                    # reset temp and counter
                    self.temp[id_num] = Point()
                    self.seq_count[id_num] = 0

                    # broadcast TF
                    self.broadcast_tf(id_num, self.real_position[id_num])

                self.is_mark_empty[id_num] = False
            else:
                rospy.logwarn(f"Invalid marker id: {id_num}")

    def broadcast_tf(self, id_num, position):
        """Broadcast a static transform for each marker."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f"marker_{id_num}"
        t.transform.translation = position
        # no rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)
        rospy.logdebug(f"TF broadcasted for marker_{id_num}")

    def handle_qr_service(self, req):
        """Service handler: returns all marker IDs and positions."""
        ids = []
        points = []
        for id_num, pos in self.real_position.items():
            if not self.is_mark_empty[id_num]:
                ids.append(id_num)
                points.append(pos)

        resp = QrDataResponse()
        resp.ids = ids
        resp.positions = points
        resp.success = True
        return resp

if __name__ == '__main__':
    try:
        QrServiceNode()
    except rospy.ROSInterruptException:
        pass
