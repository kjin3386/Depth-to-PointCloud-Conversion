#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import sensor_msgs_py.point_cloud2 as pc2

class CleanFloorObstacleDetector(Node):
    def __init__(self):
        super().__init__('clean_floor_obstacle_detector')
        self.bridge = CvBridge()
        
        # PointCloud2 Publishers
        self.obstacle_pc_publisher = self.create_publisher(PointCloud2, '/obstacles/pointcloud', 10)
        self.floor_pc_publisher = self.create_publisher(PointCloud2, '/floor/pointcloud', 10)
        
        # Depth image subscription
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.image_callback,
            10
        )
        
        # Camera info subscription
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.info_callback,
            10
        )
        
        self.cam_model = PinholeCameraModel()
        self.camera_info = None
        
        # Simple height-based parameters (camera at 50cm height, horizontal orientation)
        # You can filter the point cloud by manually setting the camera's extrinsic parameters.
        # These pre-configured parameters allow you to extract points within the 0~50cm height range 
        # in the world coordinate system.

        self.camera_height = 0.50      # Camera height from ground: 50cm
        self.floor_y_min = 0.35        # Floor Y minimum value (35cm below camera)
        self.floor_y_max = 0.65        # Floor Y maximum value (65cm below camera)  
        self.obstacle_y_min = -0.10    # Obstacle Y minimum value (10cm above camera)
        self.obstacle_y_max = 0.25     # Obstacle Y maximum value (25cm below camera)
        self.depth_min = 0.3           # Minimum depth range (m)
        self.depth_max = 4.0           # Maximum depth range (m)
        self.sampling_step = 3         # Pixel sampling interval
        
        self.get_logger().info("Clean floor-based obstacle detection node has started.")
        self.get_logger().info(f"Floor range: Y {self.floor_y_min}~{self.floor_y_max}")
        self.get_logger().info(f"Obstacle range: Y {self.obstacle_y_min}~{self.obstacle_y_max}")

    def info_callback(self, msg):
        """Callback function for camera info"""
        self.cam_model.fromCameraInfo(msg)
        self.camera_info = msg
        self.get_logger().info(f"Camera info received - fx: {msg.k[0]:.1f}, fy: {msg.k[4]:.1f}")

    def image_callback(self, msg):
        """Callback function for depth image"""
        if self.camera_info is None:
            self.get_logger().warn("Waiting for camera info...")
            return

        try:
            # Convert depth image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth_image.shape
            
            # Get camera intrinsic parameters
            fx = self.cam_model.fx()
            fy = self.cam_model.fy()
            cx = self.cam_model.cx()
            cy = self.cam_model.cy()
            
            # Phase 1: Generate all valid 3D points
            all_points = []
            valid_count = 0
            
            for v in range(0, h, self.sampling_step):
                for u in range(0, w, self.sampling_step):
                    z = depth_image[v, u] / 1000.0  # mm → meters
                    
                    if z == 0.0 or z < self.depth_min or z > self.depth_max:
                        continue
                    
                    valid_count += 1
                    
                    # Convert pixel coordinates to 3D coordinates
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    # Only consider points in front of the camera (within 2m horizontally)
                    if abs(x) < 2.0:
                        all_points.append([x, y, z])

            self.get_logger().info(f"Valid depth pixels: {valid_count}, Converted points: {len(all_points)}")

            if len(all_points) < 20:
                self.get_logger().warn(f"Insufficient valid points: {len(all_points)}")
                return

            # Phase 2: Separate floor and obstacles
            floor_points, obstacle_points = self.classify_points(all_points)
            
            # Phase 3: Publish results
            header = msg.header
            header.frame_id = 'camera_depth_optical_frame'
            
            # Publish both topics even if empty
            self.publish_pointcloud(obstacle_points, header, self.obstacle_pc_publisher)
            self.publish_pointcloud(floor_points, header, self.floor_pc_publisher)
            
            self.get_logger().info(f"Publishing completed - Obstacles: {len(obstacle_points)}, Floor: {len(floor_points)}")
            
        except Exception as e:
            self.get_logger().error(f"Error occurred during obstacle detection: {str(e)}")

    def classify_points(self, points):
        """Separate floor and obstacles based on camera height (extrinsic parameters)"""
        floor_points = []
        obstacle_points = []
        y_values = []
        
        for point in points:
            x, y, z = point
            y_values.append(y)
            
            # Classification based on height (camera reference frame)
            if self.floor_y_min <= y <= self.floor_y_max:
                # Floor area (35~65cm below camera)
                floor_points.append(point)
            elif self.obstacle_y_min <= y <= self.obstacle_y_max:
                # Obstacle area (10cm above camera ~ 25cm below camera)
                obstacle_points.append(point)
            # Ignore points outside these criteria
        
        # Debug information
        if y_values:
            y_min, y_max = min(y_values), max(y_values)
            y_mean = sum(y_values) / len(y_values)
            self.get_logger().info(f"Y distribution: {y_min:.2f}~{y_max:.2f}, Mean: {y_mean:.2f}")
        
        self.get_logger().info(f"Classification completed - Floor: {len(floor_points)}, Obstacles: {len(obstacle_points)}")
        
        return floor_points, obstacle_points

    def publish_pointcloud(self, points, original_header, publisher):
        """Publish point cloud data"""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        header = original_header
        header.frame_id = 'camera_depth_optical_frame'
        
        # Publish even empty lists (clears previous data in RViz)
        pc_msg = pc2.create_cloud(header, fields, points)
        publisher.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CleanFloorObstacleDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()