import time
import threading

import rclpy  
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import sensor_msgs.msg
from cv_bridge import CvBridge  # To convert to OpenCV image
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('rpi_node')

        self.br = CvBridge()
        self.frame_rate = 30  # Max frame rate in Hz
        self.publish_interval = 5  # Publish every 5th frame

        # Configure QoS for the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher_src_frame = self.create_publisher(
            sensor_msgs.msg.Image, 'camera/src_frame', qos_profile
        )

        self.i = 0  # Count iterations in the main loop

        # Initialize the video device
        self.cam = cv2.VideoCapture(0)
        if not self.cam.isOpened():
            self.get_logger().error("Failed to open the camera.")
            raise RuntimeError("Camera initialization failed.")
        
        # Set camera properties
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Adjust resolution
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        # self.cam.set(cv2.CAP_PROP_FPS, 60)  # Adjust FPS if supported by the device

    def loop(self):
        try:
            while rclpy.ok():
                start_time = time.perf_counter()

                self.i += 1

                # Image capture
                success, frame = self.cam.read()
                if not success:
                    self.get_logger().warning("Failed to grab frame. Skipping...")
                    continue

                # Publish every Nth frame
                if self.i % self.publish_interval == 0:
                    # Convert BGR to RGB and publish the image
                    rgb_frame = frame #rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    ros_image = self.br.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
                    self.publisher_src_frame.publish(ros_image)

                # Maintain desired frame rate
                elapsed_time = time.perf_counter() - start_time
                sleep_time = max(0, (1 / self.frame_rate) - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            # Release camera resource when exiting
            self.cam.release()
            self.get_logger().info("Camera released.")

    def destroy(self):
        """Cleanup resources explicitly."""
        self.cam.release()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    # Starts the loop in a thread because rclpy.spin is blocking
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
