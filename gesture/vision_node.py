import cv2
import time
import math
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.MODEL_PATH    = 'best.pt'
        self.CAM_INDEX     = 0
        self.ALPHA         = 0.15  
        self.MIN_MOVE_PX   = 3.0   
        self.STILL_TIMEOUT = 0.8    

        self.pub_pos    = self.create_publisher(Point, '/finger_pos',    10)
        self.pub_moving = self.create_publisher(Bool,  '/finger_moving', 10)

        self.get_logger().info(f'Loading YOLO: {self.MODEL_PATH}')
        self.model = YOLO(self.MODEL_PATH)

        self.cap = cv2.VideoCapture(self.CAM_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError('Cannot open camera!')
        time.sleep(1.0)

        self.smooth_pt      = None
        self.trail          = []
        self.last_move_time = time.time()
        self.is_moving      = False

        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)
        self.get_logger().info('VisionNode ready.')

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        frame = cv2.flip(frame, 1)   
        results = self.model(frame, conf=0.25, imgsz=640, verbose=False)

        if len(results[0].boxes) > 0:
            boxes = results[0].boxes
            best  = int(boxes.conf.argmax())
            x1, y1, x2, y2 = map(int, boxes.xyxy[best].cpu().numpy())

            raw_cx = (x1 + x2) // 2
            raw_cy = (y1 + y2) // 2

            if self.smooth_pt is None:
                self.smooth_pt = (float(raw_cx), float(raw_cy))
            else:
                sx = self.ALPHA * raw_cx + (1 - self.ALPHA) * self.smooth_pt[0]
                sy = self.ALPHA * raw_cy + (1 - self.ALPHA) * self.smooth_pt[1]
                self.smooth_pt = (sx, sy)

            cx, cy = int(self.smooth_pt[0]), int(self.smooth_pt[1])

            if self.trail:
                moved = math.hypot(cx - self.trail[-1][0],
                                   cy - self.trail[-1][1]) > self.MIN_MOVE_PX
            else:
                moved = True

            if moved:
                self.last_move_time = time.time()

            now_moving     = (time.time() - self.last_move_time) < self.STILL_TIMEOUT
            self.is_moving = now_moving

            pt = Point()
            pt.x, pt.y, pt.z = float(cx), float(cy), 0.0
            self.pub_pos.publish(pt)
            flag      = Bool()
            flag.data = now_moving
            self.pub_moving.publish(flag)

            self.trail.append((cx, cy))
            if len(self.trail) > 300:
                self.trail.pop(0)

            clr = (0, 255, 0) if now_moving else (0, 128, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), clr, 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, 'MOVING' if now_moving else 'STILL',
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, clr, 2)

        else:
            if time.time() - self.last_move_time > self.STILL_TIMEOUT:
                self.is_moving = False
                flag = Bool(); flag.data = False
                self.pub_moving.publish(flag)

        for i in range(1, len(self.trail)):
            cv2.line(frame, self.trail[i - 1], self.trail[i], (0, 200, 100), 2)

        cv2.putText(frame,
                    f'Robot: {"MOVING" if self.is_moving else "STOPPED"}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if self.is_moving else (0, 100, 255), 2)

        cv2.imshow('Finger Tracking', frame)
        if cv2.waitKey(1) == 27:
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
