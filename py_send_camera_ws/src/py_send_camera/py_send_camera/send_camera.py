import time
import threading
import numpy as np

import rclpy  
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import sensor_msgs.msg
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2


class BarycenterTracker:
    """
    Barycenter (Centroid) tracking for objects and lines.
    Uses image moments to calculate the center of mass.
    """
    
    def __init__(self):
        self.tracking_mode = "contour"  # Modes: contour, line, color, binary
        self.show_visualization = True
        
        # Color range for color-based tracking (HSV)
        # Default: Red objects
        self.lower_color = np.array([0, 120, 70])
        self.upper_color = np.array([10, 255, 255])
        self.lower_color2 = np.array([170, 120, 70])
        self.upper_color2 = np.array([180, 255, 255])
        
    def calculate_barycenter(self, binary_image):
        """
        Calculate barycenter (centroid) using image moments.
        
        Args:
            binary_image: Binary image (thresholded)
            
        Returns:
            (cx, cy): Barycenter coordinates, or None if not found
        """
        # Calculate moments
        moments = cv2.moments(binary_image)
        
        # Calculate barycenter
        if moments['m00'] > 0:  # Avoid division by zero
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            return (cx, cy)
        
        return None
    
    def track_contours(self, frame):
        """
        Track all contours and their barycenters.
        
        Returns:
            annotated_frame: Frame with barycenters marked
            barycenters: List of (x, y, area) tuples
        """
        # Convert to grayscale and threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        output = frame.copy()
        barycenters = []
        
        # Filter by minimum area
        min_area = 500
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > min_area:
                # Calculate barycenter using moments
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    barycenters.append((cx, cy, area))
                    
                    if self.show_visualization:
                        # Draw contour
                        cv2.drawContours(output, [contour], -1, (0, 255, 0), 2)
                        
                        # Draw barycenter (large circle)
                        cv2.circle(output, (cx, cy), 8, (0, 0, 255), -1)
                        
                        # Draw crosshair at barycenter
                        cv2.line(output, (cx-15, cy), (cx+15, cy), (255, 0, 0), 2)
                        cv2.line(output, (cx, cy-15), (cx, cy+15), (255, 0, 0), 2)
                        
                        # Add coordinates text
                        cv2.putText(output, f"({cx},{cy})", (cx+10, cy-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
                        # Add area text
                        cv2.putText(output, f"A:{int(area)}", (cx+10, cy+10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Add count
        cv2.putText(output, f"Objects: {len(barycenters)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return output, barycenters
    
    def track_line(self, frame, roi_height_ratio=0.3):
        """
        Track a line (for line following robots) using barycenter.
        
        Args:
            frame: Input image
            roi_height_ratio: Height of ROI from bottom (0.3 = bottom 30%)
            
        Returns:
            annotated_frame: Frame with line barycenter marked
            line_position: (cx, cy, error) tuple
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        
        # Define ROI (bottom portion of image)
        roi_start = int(height * (1 - roi_height_ratio))
        roi = gray[roi_start:height, :]
        
        # Apply adaptive threshold (better for varying lighting)
        binary = cv2.adaptiveThreshold(roi, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)
        
        # Calculate barycenter
        barycenter = self.calculate_barycenter(binary)
        
        output = frame.copy()
        
        # Draw ROI rectangle
        cv2.rectangle(output, (0, roi_start), (width, height), (255, 0, 0), 2)
        
        if barycenter:
            cx, cy = barycenter
            # Adjust cy to full frame coordinates
            cy_full = cy + roi_start
            
            # Calculate error from center
            center_x = width // 2
            error = cx - center_x
            
            # Normalize error (-1 to 1)
            normalized_error = error / (width / 2)
            
            if self.show_visualization:
                # Draw line barycenter
                cv2.circle(output, (cx, cy_full), 10, (0, 0, 255), -1)
                
                # Draw crosshair
                cv2.line(output, (cx-20, cy_full), (cx+20, cy_full), (255, 0, 0), 3)
                cv2.line(output, (cx, cy_full-20), (cx, cy_full+20), (255, 0, 0), 3)
                
                # Draw center line
                cv2.line(output, (center_x, roi_start), (center_x, height), (0, 255, 0), 2)
                
                # Draw error line
                cv2.line(output, (center_x, cy_full), (cx, cy_full), (255, 255, 0), 3)
                
                # Add error text
                cv2.putText(output, f"Error: {error}px ({normalized_error:.2f})", 
                           (10, height - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # Add position text
                cv2.putText(output, f"Line: ({cx}, {cy_full})", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            return output, (cx, cy_full, error, normalized_error)
        else:
            # No line found
            cv2.putText(output, "NO LINE DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return output, None
    
    def track_color(self, frame):
        """
        Track colored objects using barycenter.
        
        Returns:
            annotated_frame: Frame with colored object barycenters
            barycenters: List of (x, y, area) tuples
        """
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for target color
        mask1 = cv2.inRange(hsv, self.lower_color, self.upper_color)
        mask2 = cv2.inRange(hsv, self.lower_color2, self.upper_color2)
        mask = mask1 + mask2
        
        # Clean up mask with morphology
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        output = frame.copy()
        barycenters = []
        
        min_area = 300
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > min_area:
                # Calculate barycenter
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    barycenters.append((cx, cy, area))
                    
                    if self.show_visualization:
                        # Draw contour
                        cv2.drawContours(output, [contour], -1, (0, 255, 0), 2)
                        
                        # Draw barycenter
                        cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
                        
                        # Draw crosshair
                        cv2.line(output, (cx-20, cy), (cx+20, cy), (255, 0, 0), 3)
                        cv2.line(output, (cx, cy-20), (cx, cy+20), (255, 0, 0), 3)
                        
                        # Add text
                        cv2.putText(output, f"({cx},{cy}) A:{int(area)}", 
                                   (cx+15, cy-15),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Add count
        cv2.putText(output, f"Colored Objects: {len(barycenters)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return output, barycenters
    
    def track_binary_region(self, frame):
        """
        Track the barycenter of the entire binary region.
        Good for simple black/white line following.
        
        Returns:
            annotated_frame: Frame with barycenter marked
            position: (cx, cy, area) or None
        """
        # Convert to grayscale and threshold
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
        
        # Calculate barycenter of entire binary region
        barycenter = self.calculate_barycenter(binary)
        
        output = frame.copy()
        
        if barycenter:
            cx, cy = barycenter
            
            # Calculate total white pixels (area)
            white_pixels = np.count_nonzero(binary)
            
            height, width = frame.shape[:2]
            center_x = width // 2
            error = cx - center_x
            
            if self.show_visualization:
                # Draw barycenter
                cv2.circle(output, (cx, cy), 12, (0, 0, 255), -1)
                cv2.circle(output, (cx, cy), 15, (255, 255, 0), 2)
                
                # Draw crosshair
                cv2.line(output, (cx-25, cy), (cx+25, cy), (255, 0, 0), 3)
                cv2.line(output, (cx, cy-25), (cx, cy+25), (255, 0, 0), 3)
                
                # Draw center line
                cv2.line(output, (center_x, 0), (center_x, height), (0, 255, 0), 2)
                
                # Draw error line
                cv2.line(output, (center_x, cy), (cx, cy), (255, 255, 0), 3)
                
                # Add info
                cv2.putText(output, f"Barycenter: ({cx}, {cy})", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(output, f"Error: {error}px", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(output, f"Pixels: {white_pixels}", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            return output, (cx, cy, white_pixels, error)
        else:
            cv2.putText(output, "NO REGION DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return output, None


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('barycenter_camera_node')

        self.br = CvBridge()
        self.frame_rate = 30
        self.publish_interval = 5
        
        # Barycenter tracker
        self.tracker = BarycenterTracker()
        
        # Choose tracking mode:
        # "contour" - Track multiple objects
        # "line" - Track line for line following
        # "color" - Track colored objects
        # "binary" - Track binary region barycenter
        self.tracking_mode = "line"  # CHANGE THIS TO SELECT MODE
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()

        # Configure QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.publisher_processed = self.create_publisher(
            sensor_msgs.msg.Image, 'camera/src_frame', qos_profile
        )
        
        # Publisher for barycenter coordinates
        self.publisher_coordinates = self.create_publisher(
            Float32MultiArray, 'barycenter/coordinates', qos_profile
        )

        self.i = 0

        # Initialize camera
        self.cam = cv2.VideoCapture(0)
        if not self.cam.isOpened():
            self.get_logger().error("Failed to open the camera.")
            raise RuntimeError("Camera initialization failed.")
        
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.get_logger().info(f"Barycenter tracking initialized. Mode: {self.tracking_mode}")

    def loop(self):
        try:
            while rclpy.ok():
                start_time = time.perf_counter()

                self.i += 1
                self.frame_count += 1

                # Capture frame
                success, frame = self.cam.read()
                if not success:
                    self.get_logger().warning("Failed to grab frame. Skipping...")
                    continue

                # Process every Nth frame
                if self.i % self.publish_interval == 0:
                    
                    # Apply selected tracking mode
                    if self.tracking_mode == "contour":
                        processed_frame, data = self.tracker.track_contours(frame)
                    elif self.tracking_mode == "line":
                        processed_frame, data = self.tracker.track_line(frame)
                    elif self.tracking_mode == "color":
                        processed_frame, data = self.tracker.track_color(frame)
                    elif self.tracking_mode == "binary":
                        processed_frame, data = self.tracker.track_binary_region(frame)
                    else:
                        processed_frame = frame
                        data = None
                    
                    # Publish image
                    rgb_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    ros_image = self.br.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
                    self.publisher_processed.publish(ros_image)
                    
                    # Publish coordinates
                    if data is not None:
                        coord_msg = Float32MultiArray()
                        
                        if self.tracking_mode == "contour" or self.tracking_mode == "color":
                            # Multiple objects: flatten list
                            coords = []
                            for x, y, area in data:
                                coords.extend([float(x), float(y), float(area)])
                            coord_msg.data = coords
                        elif self.tracking_mode == "line":
                            # Single line: cx, cy, error, normalized_error
                            cx, cy, error, norm_error = data
                            coord_msg.data = [float(cx), float(cy), float(error), float(norm_error)]
                        elif self.tracking_mode == "binary":
                            # Binary region: cx, cy, pixels, error
                            cx, cy, pixels, error = data
                            coord_msg.data = [float(cx), float(cy), float(pixels), float(error)]
                        
                        self.publisher_coordinates.publish(coord_msg)
                        
                        # Log periodically
                        if self.frame_count % 30 == 0:
                            self.get_logger().info(f"Mode: {self.tracking_mode}, Data: {data}")

                # Maintain frame rate
                elapsed_time = time.perf_counter() - start_time
                sleep_time = max(0, (1 / self.frame_rate) - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            self.cam.release()
            self.get_logger().info("Camera released.")

    def destroy(self):
        """Cleanup resources explicitly."""
        self.cam.release()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    loop_thread = threading.Thread(target=camera_publisher.loop)
    loop_thread.start()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy()
        rclpy.shutdown()
        loop_thread.join()


if __name__ == '__main__':
    main()