#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

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

        # Инициализация EKF
        self.state = np.zeros(3)  # [x, y, theta] - положение робота
        self.covariance = np.eye(3) * 0.1  # Начальная неопределённость
        self.Q = np.diag([0.01, 0.01, 0.001])  # Шум процесса: [x, y, theta]
        self.R = np.diag([0.1, 0.1])  # Шум измерений препятствий

        # Предыдущие значения для вычисления приращений
        self.prev_odom = None
        self.prev_time = None

    def motion_model(self, delta_x, delta_y, delta_theta):
        """Модель движения робота"""
        # Вычисление нового положения
        new_x = self.state[0] + delta_x * np.cos(self.state[2]) - delta_y * np.sin(self.state[2])
        new_y = self.state[1] + delta_x * np.sin(self.state[2]) + delta_y * np.cos(self.state[2])
        new_theta = self.state[2] + delta_theta
        
        return np.array([new_x, new_y, new_theta])

    def predict(self, delta_x, delta_y, delta_theta, dt):
        """Предсказание состояния на основе одометрии"""
        # Матрица Якоби для предсказания
        F = np.eye(3)
        F[0, 2] = -delta_x * np.sin(self.state[2]) - delta_y * np.cos(self.state[2])
        F[1, 2] = delta_x * np.cos(self.state[2]) - delta_y * np.sin(self.state[2])

        # Обновление состояния
        self.state = self.motion_model(delta_x, delta_y, delta_theta)

        # Обновление ковариации
        self.covariance = F @ self.covariance @ F.T + self.Q * dt

    def update(self, z_x, z_y):
        """Коррекция на основе измерений препятствий"""
        # Предсказание измерений (преобразование препятствия в глобальные координаты)
        h = np.array([
            self.state[0] + z_x * np.cos(self.state[2]) - z_y * np.sin(self.state[2]),
            self.state[1] + z_x * np.sin(self.state[2]) + z_y * np.cos(self.state[2])
        ])

        # Матрица Якоби для измерений
        H = np.array([
            [1, 0, -z_x * np.sin(self.state[2]) - z_y * np.cos(self.state[2])],
            [0, 1, z_x * np.cos(self.state[2]) - z_y * np.sin(self.state[2])]
        ])

        # Инновация (разница между измерением и предсказанием)
        y = np.array([z_x, z_y]) - h

        # Ковариация инновации
        S = H @ self.covariance @ H.T + self.R

        # Калмановский коэффициент усиления
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Обновление состояния и ковариации
        self.state += K @ y
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

    def ekf_slam(self, odom_trans, forward_trans):
        """Основной цикл EKF SLAM"""
        current_time = rospy.Time.now()
        
        # Получение приращений из одометрии
        if self.prev_odom is not None:
            dt = (current_time - self.prev_time).to_sec()
            
            # Вычисление приращений
            delta_x = odom_trans[0] - self.prev_odom[0]
            delta_y = odom_trans[1] - self.prev_odom[1]
            delta_theta = odom_trans[2] - self.prev_odom[2]
            
            # Предсказание движения
            self.predict(delta_x, delta_y, delta_theta, dt)
            
            # Коррекция на основе измерений препятствий
            self.update(forward_trans[0], forward_trans[1])
        
        # Сохранение текущих значений
        self.prev_odom = odom_trans
        self.prev_time = current_time
        
        return self.state[0], self.state[1], self.state[2]

    def publish_ekf_tf(self, x, y, theta):
        """Публикация результатов EKF"""
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
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
                # Получение трансформаций
                (trans1, rot1) = self.listener.lookupTransform(self.parent_frame, self.child1, rospy.Time(0))
                (trans2, rot2) = self.listener.lookupTransform(self.parent_frame, self.child2, rospy.Time(0))
                
                # Извлечение угла из кватерниона
                _, _, theta1 = tf.transformations.euler_from_quaternion(rot1)
                
                # Вызов EKF SLAM
                x_ekf, y_ekf, theta_ekf = self.ekf_slam(
                    [trans1[0], trans1[1], theta1],
                    [trans2[0], trans2[1]]
                )
                
                # Публикация результатов
                self.publish_ekf_tf(x_ekf, y_ekf, theta_ekf)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle(5, "Waiting for transforms...")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = EKFNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
