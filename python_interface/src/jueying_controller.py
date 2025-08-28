#!/usr/bin/env python3
"""
Interfaz de control simple para Jueying Lite3
Conexión directa via rosbridge_websocket
Autor: Jhosep Zapata - Skaler S.A.
"""

import json
import time
import threading
from datetime import datetime
import roslibpy

class JueyingController:
    def __init__(self, host='192.168.1.117', port=9090):
        """
        Inicializa la conexión con el robot
        Args:
            host: IP del Jetson Xavier NX (cambiar según tu configuración)
            port: Puerto de rosbridge_websocket (default 9090)
        """
        self.host = host
        self.port = port
        self.client = None
        self.cmd_vel_publisher = None
        self.telemetry_data = {}
        self.is_connected = False
        
        # Control de seguridad
        self.last_cmd_time = time.time()
        self.safety_timeout = 0.25  # 250ms como en los specs
        
    def connect(self):
        """Establece conexión con rosbridge"""
        try:
            print(f"🔄 Conectando a ws://{self.host}:{self.port}")
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()
            
            # Esperar conexión
            time.sleep(1)
            
            if self.client.is_connected:
                self.is_connected = True
                print("✅ Conectado a rosbridge")
                self._setup_publishers()
                self._setup_subscribers()
                self._start_safety_monitor()
                return True
            else:
                print("❌ No se pudo conectar")
                return False
                
        except Exception as e:
            print(f"❌ Error de conexión: {e}")
            return False
    
    def _setup_publishers(self):
        """Configura publicadores ROS"""
        # Publisher para comandos de velocidad
        self.cmd_vel_publisher = roslibpy.Topic(
            self.client,
            '/cmd_vel',
            'geometry_msgs/Twist'
        )
        print("📡 Publisher /cmd_vel configurado")
        
        # Publisher para cambio de gait
        self.gait_publisher = roslibpy.Topic(
            self.client,
            '/set_gait',
            'std_msgs/String'
        )
        print("📡 Publisher /set_gait configurado")
    
    def _setup_subscribers(self):
        """Configura suscriptores para telemetría"""
        # IMU Data
        imu_listener = roslibpy.Topic(
            self.client,
            '/imu/data',
            'sensor_msgs/Imu'
        )
        imu_listener.subscribe(self._imu_callback)
        
        # Joint States
        joint_listener = roslibpy.Topic(
            self.client,
            '/joint_states',
            'sensor_msgs/JointState'
        )
        joint_listener.subscribe(self._joint_callback)
        
        # Leg Odometry
        odom_listener = roslibpy.Topic(
            self.client,
            '/leg_odom',
            'nav_msgs/Odometry'
        )
        odom_listener.subscribe(self._odom_callback)
        
        print("📡 Suscriptores de telemetría configurados")
    
    def _imu_callback(self, msg):
        """Procesa datos IMU"""
        self.telemetry_data['imu'] = {
            'orientation': msg.get('orientation', {}),
            'angular_velocity': msg.get('angular_velocity', {}),
            'linear_acceleration': msg.get('linear_acceleration', {}),
            'timestamp': datetime.now().isoformat()
        }
    
    def _joint_callback(self, msg):
        """Procesa datos de articulaciones"""
        self.telemetry_data['joints'] = {
            'names': msg.get('name', []),
            'positions': msg.get('position', []),
            'velocities': msg.get('velocity', []),
            'efforts': msg.get('effort', []),
            'timestamp': datetime.now().isoformat()
        }
    
    def _odom_callback(self, msg):
        """Procesa odometría"""
        pose = msg.get('pose', {}).get('pose', {})
        twist = msg.get('twist', {}).get('twist', {})
        self.telemetry_data['odometry'] = {
            'position': pose.get('position', {}),
            'orientation': pose.get('orientation', {}),
            'linear_velocity': twist.get('linear', {}),
            'angular_velocity': twist.get('angular', {}),
            'timestamp': datetime.now().isoformat()
        }
    
    def _start_safety_monitor(self):
        """Monitor de seguridad que detiene el robot si no hay comandos"""
        def safety_check():
            while self.is_connected:
                if time.time() - self.last_cmd_time > self.safety_timeout:
                    self.stop()
                time.sleep(0.1)
        
        safety_thread = threading.Thread(target=safety_check, daemon=True)
        safety_thread.start()
    
    def send_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """
        Envía comando de velocidad al robot
        Args:
            linear_x: Velocidad hacia adelante/atrás (m/s) [-1.0, 1.0]
            linear_y: Velocidad lateral (m/s) [-0.5, 0.5]
            angular_z: Velocidad angular (rad/s) [-1.0, 1.0]
        """
        if not self.is_connected:
            print("⚠️ No hay conexión")
            return
        
        # Limitar valores por seguridad
        linear_x = max(-1.0, min(1.0, linear_x))
        linear_y = max(-0.5, min(0.5, linear_y))
        angular_z = max(-1.0, min(1.0, angular_z))
        
        message = roslibpy.Message({
            'linear': {
                'x': linear_x,
                'y': linear_y,
                'z': 0.0
            },
            'angular': {
                'x': 0.0,
                'y': 0.0,
                'z': angular_z
            }
        })
        
        self.cmd_vel_publisher.publish(message)
        self.last_cmd_time = time.time()
        print(f"➡️ Velocidad enviada: x={linear_x:.2f}, y={linear_y:.2f}, θ={angular_z:.2f}")
    
    def stop(self):
        """Detiene inmediatamente el robot"""
        self.send_velocity(0, 0, 0)
        print("🛑 Robot detenido")
    
    def set_gait(self, gait_type):
        """
        Cambia el modo de marcha del robot
        Args:
            gait_type: 'walk', 'trot', 'crawl', etc.
        """
        if not self.is_connected:
            print("⚠️ No hay conexión")
            return
        
        message = roslibpy.Message({'data': gait_type})
        self.gait_publisher.publish(message)
        print(f"🦿 Gait cambiado a: {gait_type}")
    
    def print_telemetry(self):
        """Imprime telemetría actual"""
        print("\n📊 TELEMETRÍA DEL ROBOT:")
        print("-" * 50)
        
        # IMU
        if 'imu' in self.telemetry_data:
            imu = self.telemetry_data['imu']
            print("IMU:")
            if 'orientation' in imu:
                q = imu['orientation']
                print(f"  Orientación (quaternion): w={q.get('w',0):.3f}")
            if 'angular_velocity' in imu:
                av = imu['angular_velocity']
                print(f"  Vel. angular: x={av.get('x',0):.3f}, y={av.get('y',0):.3f}, z={av.get('z',0):.3f}")
        
        # Odometría
        if 'odometry' in self.telemetry_data:
            odom = self.telemetry_data['odometry']
            if 'position' in odom:
                pos = odom['position']
                print(f"\nPosición: x={pos.get('x',0):.3f}, y={pos.get('y',0):.3f}, z={pos.get('z',0):.3f}")
        
        # Articulaciones
        if 'joints' in self.telemetry_data:
            joints = self.telemetry_data['joints']
            if joints.get('names'):
                print(f"\nArticulaciones activas: {len(joints['names'])}")
        
        print("-" * 50)
    
    def disconnect(self):
        """Cierra la conexión"""
        if self.is_connected:
            self.stop()  # Detener robot antes de desconectar
            time.sleep(0.5)
            self.client.terminate()
            self.is_connected = False
            print("🔌 Desconectado")

def main():
    """Función principal con interfaz de comandos"""
    print("=" * 60)
    print("🤖 CONTROL JUEYING LITE3 - PRUEBA DIRECTA ROSBRIDGE")
    print("=" * 60)
    
    # Configurar IP del robot
    robot_ip = input("Ingresa la IP del robot (default: 192.168.1.117): ").strip()
    if not robot_ip:
        robot_ip = '192.168.1.117'
    
    # Crear controlador
    controller = JueyingController(host=robot_ip)
    
    # Conectar
    if not controller.connect():
        print("No se pudo establecer conexión. Verifica:")
        print("1. El robot está encendido")
        print("2. rosbridge_websocket está corriendo en el robot")
        print("3. La IP es correcta y hay conectividad")
        return
    
    print("\n✅ Sistema listo. Comandos disponibles:")
    print("-" * 40)
    print("MOVIMIENTO:")
    print("  w/s    - Adelante/Atrás")
    print("  a/d    - Girar izquierda/derecha")
    print("  q/e    - Desplazamiento lateral")
    print("  space  - Detener")
    print("\nGAIT:")
    print("  1 - Walk")
    print("  2 - Trot")
    print("  3 - Crawl")
    print("\nOTROS:")
    print("  t - Ver telemetría")
    print("  x - Salir")
    print("-" * 40)
    
    # Velocidades de movimiento
    vel_linear = 0.3  # m/s
    vel_angular = 0.5  # rad/s
    
    try:
        while True:
            cmd = input("\nComando> ").strip().lower()
            
            if cmd == 'w':
                controller.send_velocity(linear_x=vel_linear)
            elif cmd == 's':
                controller.send_velocity(linear_x=-vel_linear)
            elif cmd == 'a':
                controller.send_velocity(angular_z=vel_angular)
            elif cmd == 'd':
                controller.send_velocity(angular_z=-vel_angular)
            elif cmd == 'q':
                controller.send_velocity(linear_y=vel_linear)
            elif cmd == 'e':
                controller.send_velocity(linear_y=-vel_linear)
            elif cmd == ' ' or cmd == '':
                controller.stop()
            elif cmd == '1':
                controller.set_gait('walk')
            elif cmd == '2':
                controller.set_gait('trot')
            elif cmd == '3':
                controller.set_gait('crawl')
            elif cmd == 't':
                controller.print_telemetry()
            elif cmd == 'x':
                break
            else:
                print("Comando no reconocido")
    
    except KeyboardInterrupt:
        print("\n\nInterrumpido por usuario")
    finally:
        controller.disconnect()
        print("Programa terminado")

if __name__ == "__main__":
    main()