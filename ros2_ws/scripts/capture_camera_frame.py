#!/usr/bin/env python3
"""Capture real UAV downward-camera frames (raw) AND the pipeline's annotated debug frame
from the running SITL system. RELIABLE QoS to match the ros_gz_bridge / node publishers."""
import rclpy, cv2, os, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Output directory for captured frames. Defaults to a folder next to this script;
# override with the FRAME_CAPTURE_DIR environment variable.
OUT = os.environ.get(
    "FRAME_CAPTURE_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "captured_frames"),
)
os.makedirs(OUT, exist_ok=True)

class Cap(Node):
    def __init__(self):
        super().__init__('frame_capture')
        self.b = CvBridge(); self.nr = 0; self.nd = 0; self.lastr = 0.0; self.lastd = 0.0
        self.t0 = time.time()
        # default QoS (RELIABLE, depth 10) -> matches the publishers we probed
        self.create_subscription(Image, '/overhead_camera/image', self.raw_cb, 10)
        self.create_subscription(Image, '/middle_path/debug_image', self.dbg_cb, 10)
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.adict = d
        try: self.aparams = cv2.aruco.DetectorParameters()
        except AttributeError: self.aparams = cv2.aruco.DetectorParameters_create()
        self.aparams.minMarkerPerimeterRate = 0.01

    def raw_cb(self, msg):
        t = time.time() - self.t0
        if t < 8.0 or time.time() - self.lastr < 1.0: return
        self.lastr = time.time()
        img = self.b.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.adict, parameters=self.aparams)
        tag = 'M' if ids is not None else 'x'
        cv2.imwrite(f"{OUT}/raw_{self.nr:02d}_{tag}.png", img)
        self.get_logger().info(f"raw {self.nr} t={t:.1f} aruco={'yes' if ids is not None else 'no'}")
        self.nr += 1

    def dbg_cb(self, msg):
        t = time.time() - self.t0
        if t < 8.0 or time.time() - self.lastd < 1.0: return
        self.lastd = time.time()
        img = self.b.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(f"{OUT}/dbg_{self.nd:02d}.png", img)
        self.nd += 1

def main():
    rclpy.init(); node = Cap()
    while rclpy.ok() and (time.time()-node.t0) < 42 and (node.nr < 25):
        rclpy.spin_once(node, timeout_sec=0.3)
    print(f"captured raw={node.nr} dbg={node.nd}")
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
