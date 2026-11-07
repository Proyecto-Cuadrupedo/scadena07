import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEGS = ['front_left', 'front_right', 'rear_left', 'rear_right']
JOINTS = ['shoulder', 'leg', 'foot']
NAMES = [f'{l}_{j}' for l in LEGS for j in JOINTS]
LIMITS = {'shoulder': (-0.548, 0.548), 'leg': (-2.666, 1.548), 'foot': (-2.6, 0.1)}

STAND = [0.0, 1.067, -1.722] * 4
SIT = [0.0, 1.067, -1.722, 0.0, 1.067, -1.722, 0.0, 1.5, -2.4, 0.0, 1.5, -2.4]
POSES = {'stand': STAND, 'sit': SIT}


def clamp(value, joint):
    low, high = LIMITS[joint]
    return max(low, min(high, value))


class Gestures(Node):
    def __init__(self):
        super().__init__('gestures')
        self.pub = self.create_publisher(JointTrajectory, '/spotmicro/champ/joint_targets', 10)
        self.current = list(STAND)
        self.start = list(STAND)
        self.target = list(STAND)
        self.t0 = self.get_clock().now()
        self.duration = 2.0
        self.create_subscription(String, '/gesture', self.on_gesture, 10)
        self.create_timer(0.02, self.tick)
        self.get_logger().info('listo: stand, sit')

    def on_gesture(self, msg):
        pose = POSES.get(msg.data)
        if pose is None:
            self.get_logger().warn(f'gesto desconocido: {msg.data}')
            return
        self.start = list(self.current)
        self.target = list(pose)
        self.t0 = self.get_clock().now()
        self.get_logger().info(f'gesto: {msg.data}')

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        s = min(1.0, elapsed / self.duration)
        s = s * s * (3 - 2 * s)
        self.current = [a + (b - a) * s for a, b in zip(self.start, self.target)]
        point = JointTrajectoryPoint()
        point.positions = [clamp(v, JOINTS[i % 3]) for i, v in enumerate(self.current)]
        msg = JointTrajectory()
        msg.joint_names = NAMES
        msg.points = [point]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Gestures()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
