#!/usr/bin/env python3
"""
Simulador del message_transformer del Jueying Lite3
Versión simplificada para desarrollo
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time

class FakeMessageTransformer:
    def __init__(self):
        rospy.init_node('fake_message_transformer', anonymous=False)
        
        # Estado simulado
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.35}
        self.velocity = {'linear': {'x': 0.0, 'y': 0.0}, 'angular': {'z': 0.0}}
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_gait = "stand"
        self.last_cmd_time = rospy.Time.now()
        
        # Publishers
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.odom_pub = rospy.Publisher('/leg_odom', Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.gait_sub = rospy.Subscriber('/set_gait', String, self.gait_callback)
        
        # Timers para publicación
        rospy.Timer(rospy.Duration(0.01), self.publish_imu)  # 100Hz
        rospy.Timer(rospy.Duration(0.02), self.publish_odom)  # 50Hz
        rospy.Timer(rospy.Duration(0.02), self.publish_joints)  # 50Hz
        rospy.Timer(rospy.Duration(0.05), self.update_physics)  # 20Hz
        
        rospy.loginfo("✅ Simulador Message Transformer iniciado")
        rospy.loginfo("   Publicando: /imu/data, /leg_odom, /joint_states")
        rospy.loginfo("   Suscrito a: /cmd_vel, /set_gait")
    
    def cmd_vel_callback(self, msg):
        """Procesa comandos de velocidad"""
        self.last_cmd_time = rospy.Time.now()
        self.velocity['linear']['x'] = np.clip(msg.linear.x, -1.0, 1.0)
        self.velocity['linear']['y'] = np.clip(msg.linear.y, -0.5, 0.5)
        self.velocity['angular']['z'] = np.clip(msg.angular.z, -1.0, 1.0)
        rospy.loginfo(f"CMD: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}")
    
    def gait_callback(self, msg):
        """Procesa cambios de gait"""
        self.current_gait = msg.data
        rospy.loginfo(f"Gait: {self.current_gait}")
    
    def update_physics(self, event):
        """Física simple"""
        # Timeout de seguridad
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > 0.25:
            self.velocity['linear']['x'] = 0.0
            self.velocity['linear']['y'] = 0.0
            self.velocity['angular']['z'] = 0.0
        
        dt = 0.05
        # Actualizar posición
        self.position['x'] += self.velocity['linear']['x'] * dt * math.cos(self.orientation['yaw'])
        self.position['y'] += self.velocity['linear']['x'] * dt * math.sin(self.orientation['yaw'])
        self.orientation['yaw'] += self.velocity['angular']['z'] * dt
    
    def publish_imu(self, event):
        """Publica IMU simulado"""
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        # Quaternion simple desde yaw
        msg.orientation.w = math.cos(self.orientation['yaw'] * 0.5)
        msg.orientation.z = math.sin(self.orientation['yaw'] * 0.5)
        msg.angular_velocity.z = self.velocity['angular']['z']
        msg.linear_acceleration.z = 9.81
        
        self.imu_pub.publish(msg)
    
    def publish_odom(self, event):
        """Publica odometría simulada"""
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        
        msg.pose.pose.position.x = self.position['x']
        msg.pose.pose.position.y = self.position['y']
        msg.pose.pose.position.z = self.position['z']
        
        msg.pose.pose.orientation.w = math.cos(self.orientation['yaw'] * 0.5)
        msg.pose.pose.orientation.z = math.sin(self.orientation['yaw'] * 0.5)
        
        msg.twist.twist.linear.x = self.velocity['linear']['x']
        msg.twist.twist.linear.y = self.velocity['linear']['y']
        msg.twist.twist.angular.z = self.velocity['angular']['z']
        
        self.odom_pub.publish(msg)
    
    def publish_joints(self, event):
        """Publica joints simulados"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        
        # 12 joints (3 por pata)
        msg.name = [f"joint_{i}" for i in range(12)]
        time_now = rospy.Time.now().to_sec()
        
        for i in range(12):
            if self.current_gait == "stand":
                msg.position.append(0.0)
            else:
                # Simular movimiento
                phase = (i % 4) * math.pi / 2
                msg.position.append(0.3 * math.sin(time_now * 2 + phase))
            msg.velocity.append(0.0)
            msg.effort.append(5.0)
        
        self.joint_pub.publish(msg)

if __name__ == '__main__':
    try:
        simulator = FakeMessageTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
