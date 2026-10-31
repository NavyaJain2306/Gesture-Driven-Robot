import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import math
import time

class ControlNode(Node):
    IDLE       = 'IDLE'
    DRAWING    = 'DRAWING'
    COMPLETING = 'COMPLETING'

    def __init__(self):
        super().__init__('control_node')
        self.SCALE            = 0.005
        self.SPEED            = 0.25
        self.LD_MIN           = 0.06
        self.LD_MAX           = 0.35
        self.MAX_ANGULAR      = 3.5
        self.GOAL_TOL         = 0.06
        self.MIN_PTS          = 3
        self.PATH_DEDUP       = 0.012
        self.COMPLETE_TIMEOUT = 50
        self.MIN_SPEED        = 0.06
        self.CORNER_SLOW      = 0.10

        self.create_subscription(Point, '/finger_pos', self.finger_cb, 10)
        self.create_subscription(Bool, '/finger_moving', self.moving_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.drawn_pub = self.create_publisher(Path, '/drawn_path', 10)

        self.state           = self.IDLE
        self.path            = []
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.robot_theta     = 0.0
        self._ref_px         = None
        self._ref_py         = None
        self._ref_rx         = None
        self._ref_ry         = None
        self._complete_start = None

        self.get_logger().info("ControlNode READY")

    def moving_cb(self, msg: Bool):
        if msg.data:
            if self.state == self.IDLE:
                self.state = self.DRAWING
                self.path = []
                self._ref_px = None
                self.get_logger().info("DRAWING START")

            elif self.state == self.COMPLETING:
                self._stop()
                self.state = self.DRAWING
                self.path = []
                self._ref_px = None
                self.get_logger().info("DRAWING INTERRUPTED")

        else:
            if self.state == self.DRAWING:
                if len(self.path) >= self.MIN_PTS:
                    self.state = self.COMPLETING
                    self._complete_start = time.time()
                    self.get_logger().info("COMPLETING")
                else:
                    self._reset_to_idle("too few points")

  
    def finger_cb(self, msg: Point):
        if self.state != self.DRAWING:
            return

        px, py = msg.x, msg.y
        if self._ref_px is None:
            self._ref_px = px
            self._ref_py = py
            self._ref_rx = self.robot_x
            self._ref_ry = self.robot_y
            return

        wx = self._ref_rx + (px - self._ref_px) * self.SCALE
        wy = self._ref_ry - (py - self._ref_py) * self.SCALE

        if not self.path or \
           math.hypot(wx - self.path[-1][0], wy - self.path[-1][1]) > self.PATH_DEDUP:
            self.path.append((wx, wy))

            if len(self.path) > 300:
                self.path.pop(0)
            self._publish_drawn_path()


    def odom_cb(self, msg: Odometry):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_theta = math.atan2(siny, cosy)
        self._control_loop()


    def _control_loop(self):
        if self.state == self.IDLE:
            self._stop()
            return
        if len(self.path) < self.MIN_PTS:
            return

        fx, fy = self.path[-1]
        dist_to_end = math.hypot(fx - self.robot_x, fy - self.robot_y)

        if dist_to_end < self.GOAL_TOL and self.state == self.COMPLETING:
            self._reset_to_idle("done")
            return

        ld = self._adaptive_lookahead()
        lookahead_pt = self._find_lookahead(ld) or self.path[-1]

        lx, ly = lookahead_pt
        dx = lx - self.robot_x
        dy = ly - self.robot_y

        local_x =  math.cos(self.robot_theta)*dx + math.sin(self.robot_theta)*dy
        local_y = -math.sin(self.robot_theta)*dx + math.cos(self.robot_theta)*dy

        d = math.hypot(local_x, local_y)
        if d < 1e-6:
            return

        curvature = 2.0 * local_y / (d*d)
        curvature_norm = min(abs(curvature)/8.0, 1.0)
        corner_speed = self.CORNER_SLOW + (self.SPEED - self.CORNER_SLOW)*(1.0 - curvature_norm)

        if self.state == self.COMPLETING:
            slow_r = self.GOAL_TOL * 5.0
            v = max(corner_speed * min(dist_to_end/slow_r, 1.0), self.MIN_SPEED)
        else:
            v = corner_speed

        omega = max(-self.MAX_ANGULAR,
                    min(self.MAX_ANGULAR, v * curvature))

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.pub.publish(twist)

    
    def _adaptive_lookahead(self):
        min_d, ci = float('inf'), 0
        for i, (px, py) in enumerate(self.path):  #Loop through all points in path, finds the one closest to the robot
            d = math.hypot(px - self.robot_x, py - self.robot_y)
            if d < min_d:
                min_d, ci = d, i

        window = 6
        end = min(ci + window, len(self.path) - 1)
        if end - ci < 2:
            return (self.LD_MIN + self.LD_MAX) / 2

        total_turn = 0.0
        for i in range(ci, end - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            cx, cy = self.path[min(i + 2, len(self.path)-1)]
            v1x, v1y = bx - ax, by - ay
            v2x, v2y = cx - bx, cy - by
            total_turn += abs(v1x * v2y - v1y * v2x)   #cross product

        norm = min(total_turn / (window * 0.01 + 1e-6), 1.0) #range between 0 and 1 
        return max(self.LD_MIN, self.LD_MAX - norm * (self.LD_MAX - self.LD_MIN))  #when curvature is low, lookahead is maximum and when curvature is high, lookahead is minimum


    def _find_lookahead(self, ld):
        min_d, ci = float('inf'), 0
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - self.robot_x, py - self.robot_y)
            if d < min_d:
                min_d, ci = d, i

        for i in range(ci, len(self.path) - 1):
            ax, ay = self.path[i]
            bx, by = self.path[i+1]
            dx, dy = bx - ax, by - ay
            fx, fy = ax - self.robot_x, ay - self.robot_y
            a = dx*dx + dy*dy
            b = 2*(fx*dx + fy*dy)
            c = fx*fx + fy*fy - ld*ld

            disc = b*b - 4*a*c
            if disc < 0:
                continue
            t = (-b + math.sqrt(disc)) / (2*a)
            if 0 <= t <= 1:
                return (ax + t*dx, ay + t*dy)
        return self.path[-1] if self.path else None

    
    def _publish_drawn_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'

        for (x, y) in self.path:
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.drawn_pub.publish(msg)

    
    def _reset_to_idle(self, reason=""):
        self._stop()
        self.state = self.IDLE
        self.path = []
        self._ref_px = None
        self.get_logger().info(f"RESET: {reason}")

    
    def _stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()