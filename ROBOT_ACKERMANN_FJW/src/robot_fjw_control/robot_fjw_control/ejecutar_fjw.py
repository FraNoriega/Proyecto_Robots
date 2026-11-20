"""Recorrer F, J y W en Gazebo y dejar los trazos reales visibles en RViz.

Entrada:   /odom y /scan (Gazebo -> ros_gz_bridge -> ROS 2)
Salida:    /cmd_vel (ROS 2 -> ros_gz_bridge -> Ackermann de Gazebo)
RViz:      /trayectoria_planificada, /trayectoria_real,
           /letras_plan, /letras_recorridas, /etiquetas_fjw, /pose_actual_fjw

Los trazos F/J/W son colores intensos. Las maniobras de conexion NO se pintan
como letras; se registran en la ruta real amarilla.
"""
import csv
import math
from pathlib import Path as FilePath
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from .planning import TURN_RADIUS, Pose, make_fjw, transform_samples
from .safety import LidarGuard
from .tracking import PurePursuit

# F rojo, J verde, W azul.
COLORES = {'F': (0.97, 0.25, 0.18),
           'J': (0.16, 0.87, 0.30),
           'W': (0.19, 0.45, 1.00)}
ORDEN = 'FJW'


def yaw_de_cuaternion(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def punto(x, y, z=0.10):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


class ControlFJW(Node):
    def __init__(self):
        super().__init__('controlador_fjw')
        self.declare_parameter('max_speed', 0.12)
        self.declare_parameter('lookahead', 0.30)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('max_path_error', 0.65)
        self.declare_parameter('usar_lidar', True)
        self.declare_parameter('archivo_csv', '~/robot_fjw_ws/resultados/recorrido_fjw.csv')

        velocidad = float(self.get_parameter('max_speed').value)
        lookahead = float(self.get_parameter('lookahead').value)
        self.espera = max(0.0, float(self.get_parameter('start_delay').value))
        self.error_max = float(self.get_parameter('max_path_error').value)
        self.usar_lidar = bool(self.get_parameter('usar_lidar').value)
        self.csv_destino = str(self.get_parameter('archivo_csv').value)
        if not (0.02 <= velocidad <= 0.30):
            raise ValueError('max_speed debe estar entre 0.02 y 0.30 m/s')
        if not (0.15 <= lookahead <= 1.0):
            raise ValueError('lookahead debe estar entre 0.15 y 1.0 m')

        self.velocidad = velocidad
        self.lookahead = lookahead
        self.guardia = LidarGuard()
        self.pose = None
        self.ultima_odom = None
        self.samples = None
        self.controlador = None
        self.inicio = None
        self.terminado = False
        self.parada_error = False
        self.ultimo_aviso = 0.0
        self.ultima_muestra = 0.0
        self.ultima_publicacion = 0.0
        self.ultimo_log = 0.0
        self.ultimo_dibujo = None
        self.punto_anterior = None
        self.trazos = {letra: [] for letra in ORDEN}
        self.registros = []  # [tiempo ROS, x, y, yaw, letra, dibujando]
        self.ruta_real = Path()
        self.ruta_real.header.frame_id = 'odom'
        self.datos_guardados = False

        # Transient Local mantiene mensajes disponibles para RViz abierto tarde.
        retenido = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_plan = self.create_publisher(Path, '/trayectoria_planificada', retenido)
        self.pub_real = self.create_publisher(Path, '/trayectoria_real', retenido)
        self.pub_letras_plan = self.create_publisher(Marker, '/letras_plan', retenido)
        self.pub_letras_real = self.create_publisher(Marker, '/letras_recorridas', retenido)
        self.pub_etiquetas = self.create_publisher(Marker, '/etiquetas_fjw', retenido)
        self.pub_pose = self.create_publisher(Marker, '/pose_actual_fjw', retenido)
        self.create_subscription(Odometry, '/odom', self.recibir_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.recibir_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.actualizar)  # 20 Hz.
        self.get_logger().info('Esperando /odom. Ejecutar con Gazebo y RViz abiertos.')

    def recibir_odom(self, msg):
        pos = msg.pose.pose.position
        self.pose = Pose(pos.x, pos.y, yaw_de_cuaternion(msg.pose.pose.orientation))
        self.ultima_odom = time.monotonic()
        if self.samples is None:
            self.samples = transform_samples(make_fjw(), self.pose)
            self.controlador = PurePursuit(self.samples, self.velocidad,
                                          self.lookahead, TURN_RADIUS)
            self.inicio = time.monotonic()
            self.publicar_plan()
            largo = sum(math.hypot(b.x-a.x, b.y-a.y)
                        for a, b in zip(self.samples, self.samples[1:]))
            self.get_logger().info(
                'Ruta preparada: F -> J -> W | %d puntos | %.1f m (incluye conexiones).'
                % (len(self.samples), largo))

    def recibir_scan(self, msg):
        self.guardia.update(msg.ranges, msg.angle_min, msg.angle_increment,
                            msg.range_min, msg.range_max)

    def pose_stamped(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def marcador(self, letra, previsto):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'plan_' + letra if previsto else 'real_' + letra
        m.id = ORDEN.index(letra)
        m.action = Marker.ADD
        m.type = Marker.LINE_LIST  # No une trazos separados de la misma letra.
        m.pose.orientation.w = 1.0
        m.scale.x = 0.045 if previsto else 0.11
        m.color.r, m.color.g, m.color.b = COLORES[letra]
        m.color.a = 0.36 if previsto else 1.0
        # lifetime=0 (por defecto): los trazos no desaparecen.
        return m

    def etiqueta(self, texto, pos, identificador, tipo=Marker.TEXT_VIEW_FACING):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'etiquetas', identificador, tipo, Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position = punto(pos.x, pos.y, 0.32)
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0
        m.scale.z = 0.36
        m.text = texto
        return m

    def publicar_plan(self):
        plan = Path()
        plan.header.frame_id = 'odom'
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.poses = [self.pose_stamped(a.x, a.y, a.yaw) for a in self.samples]
        self.pub_plan.publish(plan)

        for letra in ORDEN:
            m = self.marcador(letra, previsto=True)
            for anterior, actual in zip(self.samples, self.samples[1:]):
                if anterior.draw and actual.draw and anterior.letter == letra and actual.letter == letra:
                    m.points.extend([punto(anterior.x, anterior.y, 0.07),
                                     punto(actual.x, actual.y, 0.07)])
            self.pub_letras_plan.publish(m)

        # Posiciones de las etiquetas, transformadas desde la ruta original.
        for i, letra in enumerate(ORDEN):
            trazos = [s for s in self.samples if s.draw and s.letter == letra]
            centro = Pose(sum(p.x for p in trazos) / len(trazos),
                          max(p.y for p in trazos) + 0.50, 0.0)
            self.pub_etiquetas.publish(self.etiqueta(letra, centro, i))

    def registrar_posicion(self, ahora):
        if self.controlador is None or self.pose is None:
            return
        if ahora - self.ultima_muestra < 0.10:
            return
        self.ultima_muestra = ahora

        x, y, yaw = self.pose.x, self.pose.y, self.pose.yaw
        if self.ruta_real.poses:
            p = self.ruta_real.poses[-1].pose.position
            if math.hypot(x-p.x, y-p.y) < 0.018 and not self.terminado:
                return  # Reduce el tamaño de los mensajes sin perder el trazo.

        self.ruta_real.poses.append(self.pose_stamped(x, y, yaw))
        idx = min(self.controlador.progress, len(self.samples)-1)
        referencia = self.samples[idx]
        letra = referencia.letter if referencia.draw else ''
        actual = punto(x, y, 0.12)
        # Conserva un tramo solamente cuando es el mismo trazo consecutivo.
        if letra and letra == self.ultimo_dibujo and self.punto_anterior is not None:
            if math.hypot(x-self.punto_anterior.x, y-self.punto_anterior.y) <= 0.22:
                self.trazos[letra].extend([self.punto_anterior, actual])
        self.ultimo_dibujo = letra
        self.punto_anterior = actual if letra else None
        reloj = self.get_clock().now().nanoseconds * 1e-9
        self.registros.append((round(reloj, 3), x, y, yaw, letra, bool(letra)))

    def publicar_recorrido(self):
        self.ruta_real.header.stamp = self.get_clock().now().to_msg()
        self.pub_real.publish(self.ruta_real)
        for letra in ORDEN:
            m = self.marcador(letra, previsto=False)
            m.points = list(self.trazos[letra])
            self.pub_letras_real.publish(m)

        if self.pose is not None:
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'robot_fjw', 0, Marker.ARROW, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = 0.07, 0.16, 0.19
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.83, 0.05, 1.0
            a = self.pose
            m.points = [punto(a.x, a.y, 0.18),
                        punto(a.x + 0.45*math.cos(a.yaw),
                              a.y + 0.45*math.sin(a.yaw), 0.18)]
            self.pub_pose.publish(m)

    def guardar_csv(self):
        if self.datos_guardados or not self.registros or not self.csv_destino:
            return
        try:
            salida = FilePath(self.csv_destino).expanduser()
            salida.parent.mkdir(parents=True, exist_ok=True)
            with salida.open('w', newline='', encoding='utf-8') as archivo:
                writer = csv.writer(archivo)
                writer.writerow(['tiempo_sim_s', 'x_m', 'y_m', 'yaw_rad',
                                 'letra', 'dibujando'])
                writer.writerows(self.registros)
            self.datos_guardados = True
            self.get_logger().info('Recorrido real guardado en: %s' % salida)
        except OSError as error:
            self.get_logger().error('No se pudo guardar el CSV: %s' % error)

    def detener(self):
        self.pub_cmd.publish(Twist())

    def actualizar(self):
        ahora = time.monotonic()
        if self.pose is None or self.controlador is None:
            self.detener()
            return
        self.registrar_posicion(ahora)
        if ahora - self.ultima_publicacion >= 0.25:
            self.publicar_recorrido()
            self.ultima_publicacion = ahora

        if self.terminado or self.parada_error:
            self.detener()
            return
        if self.ultima_odom is None or ahora-self.ultima_odom > 1.0:
            self.detener()
            if ahora - self.ultimo_aviso > 4.0:
                self.get_logger().warn('Sin /odom reciente; robot detenido.')
                self.ultimo_aviso = ahora
            return
        if ahora-self.inicio < self.espera:
            self.detener()
            return
        if self.usar_lidar:
            escala, motivo = self.guardia.factor(ahora)
            if escala <= 0:
                self.detener()
                if ahora - self.ultimo_aviso > 4.0:
                    self.get_logger().warn('Detenido por LiDAR: %s' % motivo)
                    self.ultimo_aviso = ahora
                return
        else:
            escala = 1.0

        v, omega = self.controlador.update(self.pose.x, self.pose.y, self.pose.yaw)
        if self.controlador.done:
            self.terminado = True
            self.detener()
            self.publicar_recorrido()
            self.guardar_csv()
            self.get_logger().info('¡F, J y W TERMINADAS! RViz conserva los trazos.')
            return
        if self.controlador.lateral_error > self.error_max:
            self.parada_error = True
            self.detener()
            self.guardar_csv()
            self.get_logger().error(
                'PARADA: el robot se separó %.2f m de la ruta (límite %.2f m). '
                'Reinicia Gazebo antes de otra prueba.'
                % (self.controlador.lateral_error, self.error_max))
            return

        comando = Twist()
        comando.linear.x = v * escala
        comando.angular.z = omega * escala
        self.pub_cmd.publish(comando)
        if ahora-self.ultimo_log > 10.0:
            referencia = self.samples[self.controlador.progress]
            etiqueta = referencia.letter if referencia.draw else 'conexion'
            self.get_logger().info('Recorriendo: %s | v=%.2f m/s | error=%.2f m'
                                   % (etiqueta, comando.linear.x,
                                      self.controlador.lateral_error))
            self.ultimo_log = ahora


def main(args=None):
    rclpy.init(args=args)
    nodo = ControlFJW()
    try:
        rclpy.spin(nodo)  # No salir al completar: mantiene RViz y publica velocidad 0.
    except KeyboardInterrupt:
        pass
    finally:
        nodo.detener()
        nodo.publicar_recorrido()
        nodo.guardar_csv()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
