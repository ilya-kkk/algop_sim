#!/usr/bin/env python3
import os
os.environ['NO_AT_BRIDGE'] = '1'  # Отключаем accessibility bus
import rospy
import smach
import smach_ros
import tf
import math
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# Константы для управления
LINEAR_SPEED = 1.2  # м/с
ANGULAR_SPEED = 0.5  # рад/с
DISTANCE_THRESHOLD = 0.5  # м
ANGLE_THRESHOLD = math.pi/2  # 90 градусов
Y_THRESHOLD = 0.1  # м
START_DELAY = 5.0  # Задержка перед стартом в секундах
FINAL_DRIVE_TIME = 5.0  # Время финального движения вперёд

class DriveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_detected'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        rospy.loginfo('Executing state DRIVE_FORWARD')
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            try:
                (trans, _) = self.tf_listener.lookupTransform('base_link', 'forward', rospy.Time(0))
                distance = math.sqrt(trans[0]**2 + trans[1]**2)
                
                if distance < DISTANCE_THRESHOLD:
                    return 'obstacle_detected'
                
                # Публикация команды движения
                cmd = Twist()
                cmd.linear.x = LINEAR_SPEED
                self.cmd_vel_pub.publish(cmd)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('TF exception in DriveForward')
            
            rate.sleep()

class TurnRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turned'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.initial_yaw = None

    def execute(self, userdata):
        rospy.loginfo('Executing state TURN_RIGHT')
        rate = rospy.Rate(10)
        
        # Получение начального угла
        try:
            (_, rot) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
            _, _, self.initial_yaw = euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('TF exception in TurnRight')
            return 'turned'
        
        target_yaw = self.initial_yaw - ANGLE_THRESHOLD
        
        while not rospy.is_shutdown():
            try:
                (_, rot) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
                _, _, current_yaw = euler_from_quaternion(rot)
                
                if abs(current_yaw - target_yaw) < 0.1:
                    return 'turned'
                
                # Публикация команды поворота
                cmd = Twist()
                cmd.angular.z = -ANGULAR_SPEED
                self.cmd_vel_pub.publish(cmd)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('TF exception in TurnRight')
            
            rate.sleep()

class DriveCircle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['circle_completed'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.initial_y = None

    def execute(self, userdata):
        rospy.loginfo('Executing state DRIVE_CIRCLE')
        rate = rospy.Rate(10)
        
        # Получение начальной Y координаты
        try:
            (trans, _) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
            self.initial_y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('TF exception in DriveCircle')
            return 'circle_completed'
        
        while not rospy.is_shutdown():
            try:
                (trans, _) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
                current_y = trans[1]
                
                if abs(current_y - self.initial_y) < Y_THRESHOLD:
                    return 'circle_completed'
                
                # Публикация команды движения по кругу
                cmd = Twist()
                cmd.linear.x = LINEAR_SPEED
                cmd.angular.z = ANGULAR_SPEED
                self.cmd_vel_pub.publish(cmd)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('TF exception in DriveCircle')
            
            rate.sleep()

class TurnLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turned'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.initial_yaw = None

    def execute(self, userdata):
        rospy.loginfo('Executing state TURN_LEFT')
        rate = rospy.Rate(10)
        
        # Получение начального угла
        try:
            (_, rot) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
            _, _, self.initial_yaw = euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('TF exception in TurnLeft')
            return 'turned'
        
        target_yaw = self.initial_yaw + ANGLE_THRESHOLD
        
        while not rospy.is_shutdown():
            try:
                (_, rot) = self.tf_listener.lookupTransform('base_link', 'odom', rospy.Time(0))
                _, _, current_yaw = euler_from_quaternion(rot)
                
                if abs(current_yaw - target_yaw) < 0.1:
                    return 'turned'
                
                # Публикация команды поворота
                cmd = Twist()
                cmd.angular.z = ANGULAR_SPEED
                self.cmd_vel_pub.publish(cmd)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('TF exception in TurnLeft')
            
            rate.sleep()

class FinalDrive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.start_time = None

    def execute(self, userdata):
        rospy.loginfo('Executing state FINAL_DRIVE')
        self.start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            
            if elapsed >= FINAL_DRIVE_TIME:
                # Остановка робота
                cmd = Twist()
                self.cmd_vel_pub.publish(cmd)
                return 'finished'
            
            # Публикация команды движения
            cmd = Twist()
            cmd.linear.x = LINEAR_SPEED
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()

def main():
    rospy.init_node('fsm_node')
    
    # Создание конечного автомата
    sm = smach.StateMachine(outcomes=['finish'])
    
    with sm:
        # Добавление состояний с указанием начального состояния
        smach.StateMachine.add('DRIVE_FORWARD', DriveForward(),
                             transitions={'obstacle_detected': 'TURN_RIGHT'},
                             remapping={})
        
        smach.StateMachine.add('TURN_RIGHT', TurnRight(),
                             transitions={'turned': 'DRIVE_CIRCLE'},
                             remapping={})
        
        smach.StateMachine.add('DRIVE_CIRCLE', DriveCircle(),
                             transitions={'circle_completed': 'TURN_LEFT'},
                             remapping={})
        
        smach.StateMachine.add('TURN_LEFT', TurnLeft(),
                             transitions={'turned': 'FINAL_DRIVE'},
                             remapping={})
        
        smach.StateMachine.add('FINAL_DRIVE', FinalDrive(),
                             transitions={'finished': 'finish'},
                             remapping={})
    
    # Создание и запуск сервера интроспекции
    sis = smach_ros.IntrospectionServer('fsm_server', sm, '/FSM')
    sis.start()
    
    # Задержка перед запуском
    rospy.loginfo(f'Waiting {START_DELAY} seconds before starting...')
    rospy.sleep(START_DELAY)
    rospy.loginfo('Starting state machine...')
    
    # Запуск конечного автомата с указанием начального состояния
    outcome = sm.execute()
    
    # Остановка сервера интроспекции
    sis.stop()

if __name__ == '__main__':
    main()