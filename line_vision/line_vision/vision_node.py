import time

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

# Per-frame timing columns; must match the C++ node so results are comparable.
TIMING_HEADER = 'convert_us,process_us,publish_us,debug_us,total_us,found'


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # show_debug: draw overlay + imshow windows (off for benchmarking)
        # timing_csv: if set, write per-frame stage timings there on shutdown
        self.show_debug = self.declare_parameter('show_debug', True).value
        self.timing_csv = self.declare_parameter('timing_csv', '').value
        self.timing_rows = []

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.error_publisher = self.create_publisher(
            Float32,
            '/line_error',
            10
        )

    def image_callback(self, msg):
        t0 = time.perf_counter_ns()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return
        t1 = time.perf_counter_ns()

        h, w = frame.shape[:2]
        roi_top = int(h * 0.75)      # keep bottom 25%
        roi = frame[roi_top:h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([100, 100, 50])
        upper = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        M = cv2.moments(mask)

        found = M["m00"] != 0
        if found:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            offset = cx - (w // 2)
            error = offset/(w/2.0)
            error = max(-1.0, min(1.0, error))
        t2 = time.perf_counter_ns()

        if found:
            error_msg = Float32()
            error_msg.data = float(error)

            self.error_publisher.publish(error_msg)
            self.get_logger().info(
                f"center=({cx},{cy}), offset={offset}, error={error:.3f}",
                throttle_duration_sec=0.5,
            )
        else:
            self.get_logger().warn(
                "not seen line",
                throttle_duration_sec=1.0,
            )
        t3 = time.perf_counter_ns()

        if self.show_debug:
            cv2.rectangle(frame, (0, roi_top), (w - 1, h - 1), (255, 0, 0), 2)
            cv2.line(frame, (w // 2, roi_top), (w // 2, h), (0, 255, 0), 2)
            if found:
                cv2.circle(frame, (cx, cy + roi_top), 6, (0, 0, 255), -1)
                cv2.putText(frame, f"offset={offset}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow('Camera', frame)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)
        t4 = time.perf_counter_ns()

        if self.timing_csv:
            self.timing_rows.append(
                f'{(t1 - t0) / 1e3:.1f},{(t2 - t1) / 1e3:.1f},{(t3 - t2) / 1e3:.1f},'
                f'{(t4 - t3) / 1e3:.1f},{(t4 - t0) / 1e3:.1f},{int(found)}')

    def write_timing(self):
        if not self.timing_csv:
            return
        with open(self.timing_csv, 'w') as f:
            f.write(TIMING_HEADER + '\n')
            f.write('\n'.join(self.timing_rows) + '\n')


def main(My_args=None):
    rclpy.init(args=My_args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_timing()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
