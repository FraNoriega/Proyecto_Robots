"""Short and cautious ROS/Gazebo test: straight -> left -> right.

Stop and inspect results BEFORE running the F/J/W follower.
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from .planning import Pose
from .safety import LidarGuard


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


class TestMovimiento(Node):
    def __init__(self):
        super().__init__('prueba_movimiento_ackermann')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, 10)
        self.create_timer(.05, self.tick)
        self.guard = LidarGuard()
        self.pose = None
        self.odom_time = None
        self.stage = -1
        self.stage_start = None
        self.stage_pose = None
        self.finished = False
        self.get_logger().info('PRUEBA: avance recto, giro izquierda, giro derecha. '
                               'No ejecutar a la vez que ejecutar_fjw.')

    def odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = Pose(p.x, p.y, yaw(q))
        self.odom_time = time.monotonic()

    def scan(self, m):
        self.guard.update(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max)

    def stop(self, cause, ok=False):
        if not self.finished:
            (self.get_logger().info if ok else self.get_logger().error)(cause)
            self.get_logger().info('FIN PRUEBA; se manda velocidad CERO. Interrumpir con Ctrl+C.')
        self.finished = True
        self.pub.publish(Twist())

    def begin(self, stage, now):
        self.stage = stage
        self.stage_start = now
        self.stage_pose = self.pose
        desc = ('RECTO', 'IZQUIERDA', 'DERECHA')[stage]
        self.get_logger().info('ETAPA %d/3: %s' % (stage+1, desc))

    def tick(self):
        if self.finished:
            self.pub.publish(Twist())
            return
        now = time.monotonic()
        if self.pose is None or self.odom_time is None or now-self.odom_time > 1.5:
            self.pub.publish(Twist())
            self.get_logger().warn('Esperando odometría /odom.', throttle_duration_sec=3.)
            return
        factor, reason = self.guard.factor(now)
        if factor <= 0:
            # Missing LiDAR is allowed briefly at startup, but not during a stage.
            if self.stage >= 0:
                self.stop('PROTECCIÓN ACTIVADA: ' + reason)
            else:
                self.get_logger().warn(reason, throttle_duration_sec=3.)
                self.pub.publish(Twist())
            return
        if self.stage < 0:
            self.begin(0, now)
        dt = now-self.stage_start
        if dt >= 3.5:
            dx = self.pose.x-self.stage_pose.x
            dy = self.pose.y-self.stage_pose.y
            dyaw = wrap(self.pose.yaw-self.stage_pose.yaw)
            forward = dx*math.cos(self.stage_pose.yaw)+dy*math.sin(self.stage_pose.yaw)
            self.get_logger().info('Resultado %d: avance proyectado=%.3f m, '
                                   'desplazamiento=%.3f m, giro=%.1f grados' %
                                   (self.stage+1, forward, math.hypot(dx,dy), math.degrees(dyaw)))
            if self.stage == 0 and (forward < .12 or abs(dyaw) > .30):
                self.stop('FALLA EN RECTO: ruedas/joints/tracción/odometría incorrectos.')
                return
            if self.stage == 1 and dyaw < .10:
                self.stop('FALLA EN IZQUIERDA: comprobar dirección de los joints y puente /cmd_vel.')
                return
            if self.stage == 2:
                if dyaw > -.10:
                    self.stop('FALLA EN DERECHA: comprobar articulaciones de dirección.')
                else:
                    self.get_logger().info('PRUEBA APROBADA: recto/izquierda/derecha responden a /cmd_vel.')
                    self.stop('Prueba terminada correctamente.', ok=True)
                return
            self.begin(self.stage+1, now)
        msg = Twist()
        msg.linear.x = .12*factor
        msg.angular.z = (0., .14, -.14)[self.stage]*factor
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestMovimiento()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
