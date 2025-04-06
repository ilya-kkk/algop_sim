#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import TransformStamped

class EKFNode:
    def __init__(self):
        rospy.init_node("ekf_slam_node")
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)  # 10 Гц

        # Названия кадров
        self.parent_frame = "base_link"
        self.child1 = "odom"
        self.child2 = "forward"
        self.ekf_frame = "ekf"

    def ekf_slam(self, x1, y1, x2, y2):
        # Здесь будет EKF SLAM
        # Пока просто возврат центра между точками
        x = (x1 + x2) / 2.0
        y = (y1 + y2) / 2.0
        return x, y

    def publish_ekf_tf(self, x, y):
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.broadcaster.sendTransform(
            (x, y, 0),
            quat,
            rospy.Time.now(),
            self.ekf_frame,
            self.parent_frame
        )

    def run(self):
        while not rospy.is_shutdown():
            try:
                (trans1, _) = self.listener.lookupTransform(self.parent_frame, self.child1, rospy.Time(0))
                (trans2, _) = self.listener.lookupTransform(self.parent_frame, self.child2, rospy.Time(0))

                x1, y1 = trans1[0], trans1[1]
                x2, y2 = trans2[0], trans2[1]

                x_ekf, y_ekf = self.ekf_slam(x1, y1, x2, y2)
                self.publish_ekf_tf(x_ekf, y_ekf)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle(5, "Waiting for transforms...")

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = EKFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
