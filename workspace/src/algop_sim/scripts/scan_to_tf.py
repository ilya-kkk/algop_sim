#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import LaserScan
import math


def scan_callback(scan_msg):
    # Проверяем, что массив ranges содержит не менее 4 элементов
    if len(scan_msg.ranges) < 4:
        rospy.logwarn("Массив ranges содержит меньше 4 значений!")
        return

    # Извлекаем первые 4 значения
    backward_val = 0 if math.isinf(scan_msg.ranges[3]) else scan_msg.ranges[3]
    right_val    = 0 if math.isinf(scan_msg.ranges[2]) else scan_msg.ranges[2]  # вправо
    forward_val  = 0 if math.isinf(scan_msg.ranges[1]) else scan_msg.ranges[1]  # вперёд
    left_val     = 0 if math.isinf(scan_msg.ranges[0]) else scan_msg.ranges[0]  # влево

    # Используем единичный кватернион (без вращения)
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    current_time = rospy.Time.now()

    # Публикуем tf-преобразования от базового кадра "base_link"
    # Преобразование для направления назад (backward): смещение по -X
    br.sendTransform(( -backward_val, 0, 0 ),
                     quat,
                     current_time,
                     "backward",
                     "base_link")
    # Преобразование для направления вправо (right): смещение по -Y
    br.sendTransform(( 0, right_val, 0 ),
                     quat,
                     current_time,
                     "right",
                     "base_link")
    # Преобразование для направления вперёд (forward): смещение по +X
    br.sendTransform(( forward_val, 0, 0 ),
                     quat,
                     current_time,
                     "forward",
                     "base_link")
    # Преобразование для направления влево (left): смещение по +Y
    br.sendTransform(( 0, -left_val, 0 ),
                     quat,
                     current_time,
                     "left",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('tf_from_scan_node')
    # Инициализируем tf broadcaster
    br = tf.TransformBroadcaster()
    # Подписываемся на топик /scan
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.loginfo("Нода tf_from_scan_node запущена и слушает топик /scan")
    rospy.spin()
