#!/usr/bin/env python3
"""
Robot Vision Node - GStreamer Fix
Forces OpenCV to use V4L2 backend instead of GStreamer
"""

import time
import threading
import numpy as np
import cv2
import sys
import os

# CRITICAL: Force OpenCV to use V4L2 backend instead of GStreamer
os.environ["OPENCV_VIDEOIO_PRIORITY_MSMF"] = "0"
os.environ["OPENCV_VIDEOIO_PRIORITY_V4L2"] = "1"
os.environ["OPENCV_VIDEOIO_PRIORITY_GSTREAMER"] = "0"

import rclpy  
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import sensor_msgs.msg
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge


def find_working_camera(max_tries=10):
    """
    Find working camera using V4L2 backend directly
    """
    print("Searching for working camera with V4L2 backend...")
    
    for i in range(max_tries):
        print(f"  Trying camera index {i}...", end=" ", flush=True)
        
        # Force V4L2 backend
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        
        if cap.isOpened():
            # Give camera time to initialize
            time.sleep(0.3)
            
            # Try multiple frame reads
            success = False
            for attempt in range(3):
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    h, w = frame.shape[:2]
                    print(f"✓ Working! ({w}x{h})")
                    return i, cap
                time.sleep(0.1)
            
            print("✗ Opens but can't read frames")
            cap.release()
        else:
            print("✗ Can't open")
    
    print("\n⚠ No working camera found!")
    return None, None


class OptimizedVisionTracker:
    """Optimized vision tracking"""
    
    def __init__(self):
        self.tracking_mode = "contour"
        self.show_visualization = True
        
        # Contour parameters
        self.min_contour_area = 500
        self.max_contour_area = 50000
        
        # Kernels
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_medium = np.ones((5, 5), np.uint8)
        self.kernel_large = np.ones((7, 7), np.uint8)
    
    def preprocess_for_contours(self, frame):
        """Advanced preprocessing"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)
        binary = cv2.adaptiveThreshold(filtered, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernel_small)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel_medium)
        
        try:
            # Watershed for separation
            dist_transform = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
            _, sure_fg = cv2.threshold(dist_transform, 0.3 * dist_transform.max(), 255, 0)
            sure_fg = np.uint8(sure_fg)
            sure_bg = cv2.dilate(binary, self.kernel_large, iterations=1)
            unknown = cv2.subtract(sure_bg, sure_fg)
            _, markers = cv2.connectedComponents(sure_fg)
            markers = markers + 1
            markers[unknown == 255] = 0
            frame_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            markers = cv2.watershed(frame_color, markers)
            result = np.zeros_like(binary)
            result[markers > 1] = 255
            return result
        except:
            return binary
    
    def track_contours_optimized(self, frame):
        """Optimized contour tracking"""
        binary = self.preprocess_for_contours(frame)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        output = frame.copy()
        detections = []
        
        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if self.min_contour_area < area < self.max_contour_area:
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    detections.append({'cx': cx, 'cy': cy, 'area': area, 'bbox': (x, y, w, h)})
                    
                    if self.show_visualization:
                        color = self.generate_color(i)
                        cv2.drawContours(output, [contour], -1, color, 2)
                        cv2.rectangle(output, (x, y), (x+w, y+h), color, 1)
                        cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)
                        cv2.line(output, (cx-12, cy), (cx+12, cy), (255, 0, 0), 2)
                        cv2.line(output, (cx, cy-12), (cx, cy+12), (255, 0, 0), 2)
                        cv2.putText(output, f"#{i+1}", (cx+10, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        cv2.putText(output, f"Objects: {len(detections)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return output, detections
    
    @staticmethod
    def generate_color(index):
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        return colors[index % len(colors)]


class RobotVisionNode(Node):
    """Vision node with V4L2 backend"""
    
    def __init__(self):
        super().__init__('robot_vision_node')
        
        # Parameters
        self.declare_parameter('tracking_mode', 'contour')
        self.declare_parameter('camera_index', -1)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('frame_rate', 15)  # Lower rate for stability
        
        self.tracking_mode = self.get_parameter('tracking_mode').value
        camera_index = self.get_parameter('camera_index').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        self.frame_rate = self.get_parameter('frame_rate').value
        
        # Tracker
        self.tracker = OptimizedVisionTracker()
        self.tracker.tracking_mode = self.tracking_mode
        
        # CV Bridge
        self.br = CvBridge()
        
        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.publisher_raw = self.create_publisher(sensor_msgs.msg.Image, '/camera/raw_frame', qos_profile)
        self.publisher_processed = self.create_publisher(sensor_msgs.msg.Image, '/camera/src_frame', qos_profile)
        self.publisher_detections = self.create_publisher(Float32MultiArray, '/vision/detections', 10)
        
        # Initialize camera with V4L2
        self.cam = None
        if camera_index == -1:
            detected_index, self.cam = find_working_camera()
            if self.cam is None:
                self.get_logger().error("NO CAMERA FOUND!")
                self.get_logger().error("The diagnostic found camera 1, but it's not working properly.")
                self.get_logger().error("")
                self.get_logger().error("Try these fixes:")
                self.get_logger().error("1. Unplug and replug the USB camera")
                self.get_logger().error("2. Try a different USB port")
                self.get_logger().error("3. Reboot the system")
                self.get_logger().error("4. Check if another program is using the camera")
                raise RuntimeError("Camera initialization failed")
            camera_index = detected_index
        else:
            # Use V4L2 backend explicitly
            self.cam = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            if not self.cam.isOpened():
                self.get_logger().error(f"Failed to open camera {camera_index} with V4L2")
                raise RuntimeError(f"Camera {camera_index} not available")
            
            # Wait for camera to initialize
            time.sleep(0.5)
        
        # Configure camera
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cam.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Force MJPEG format (more compatible)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Set buffer size to 1 (reduce latency)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Warm up camera - read and discard first few frames
        self.get_logger().info("Warming up camera...")
        for _ in range(10):
            self.cam.read()
            time.sleep(0.1)
        
        # Verify camera works
        ret, test_frame = self.cam.read()
        if not ret or test_frame is None:
            self.get_logger().error("Camera opened but cannot read frames!")
            self.get_logger().error("")
            self.get_logger().error("This USB camera may be incompatible or faulty.")
            self.get_logger().error("Try:")
            self.get_logger().error("1. Different USB camera")
            self.get_logger().error("2. Different USB port (USB 2.0 port)")
            self.get_logger().error("3. Check: v4l2-ctl -d /dev/video1 --all")
            raise RuntimeError("Camera read failure")
        
        actual_width = int(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✓ CAMERA INITIALIZED SUCCESSFULLY")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Backend: V4L2 (not GStreamer)")
        self.get_logger().info(f"Camera Index: {camera_index}")
        self.get_logger().info(f"Resolution: {actual_width}x{actual_height}")
        self.get_logger().info(f"Tracking Mode: {self.tracking_mode}")
        self.get_logger().info("=" * 60)
        
        # Performance
        self.frame_count = 0
        self.failed_reads = 0
        self.start_time = time.time()
        self.processing_times = []
    
    def process_frame(self):
        """Process one frame"""
        start_time = time.perf_counter()
        
        # Capture frame
        success, frame = self.cam.read()
        if not success or frame is None:
            self.failed_reads += 1
            if self.failed_reads > 10:
                self.get_logger().error(f"Too many failed reads ({self.failed_reads}). Camera may have disconnected.")
            return
        
        self.failed_reads = 0  # Reset counter on success
        self.frame_count += 1
        
        # Publish raw
        try:
            raw_msg = self.br.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding="rgb8")
            self.publisher_raw.publish(raw_msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing raw: {e}")
        
        # Process
        try:
            if self.tracker.tracking_mode == "contour":
                processed_frame, detections = self.tracker.track_contours_optimized(frame)
            else:
                processed_frame = frame
                detections = None
        except Exception as e:
            self.get_logger().warn(f"Error in tracking: {e}")
            processed_frame = frame
            detections = None
        
        # Publish processed
        try:
            processed_msg = self.br.cv2_to_imgmsg(cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB), encoding="rgb8")
            self.publisher_processed.publish(processed_msg)
        except Exception as e:
            self.get_logger().warn(f"Error publishing processed: {e}")
        
        # Publish detections
        if detections:
            try:
                detection_msg = Float32MultiArray()
                coords = []
                for det in detections:
                    coords.extend([float(det['cx']), float(det['cy']), float(det['area'])])
                detection_msg.data = coords
                self.publisher_detections.publish(detection_msg)
            except Exception as e:
                self.get_logger().warn(f"Error publishing detections: {e}")
        
        # Performance
        processing_time = time.perf_counter() - start_time
        self.processing_times.append(processing_time)
        
        if self.frame_count % 100 == 0:
            avg_time = np.mean(self.processing_times[-100:])
            fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info(f"Performance: {fps:.1f} FPS, Processing: {avg_time*1000:.1f}ms")
    
    def run(self):
        """Main loop"""
        rate = self.create_rate(self.frame_rate)
        self.get_logger().info("Starting vision processing...")
        
        try:
            while rclpy.ok():
                self.process_frame()
                rate.sleep()
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down...")
        finally:
            if self.cam:
                self.cam.release()
            self.get_logger().info("Camera released")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        vision_node = RobotVisionNode()
        vision_thread = threading.Thread(target=vision_node.run)
        vision_thread.start()
        rclpy.spin(vision_node)
    except RuntimeError as e:
        print(f"\n{str(e)}")
        print("\nPlease fix the camera issue and try again.")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if 'vision_node' in locals():
            vision_node.destroy_node()
        rclpy.shutdown()
        if 'vision_thread' in locals():
            vision_thread.join()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())