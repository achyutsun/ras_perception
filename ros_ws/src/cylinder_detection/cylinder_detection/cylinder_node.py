# ================================================
# Achyut Shrestha   | Perception | 3D Point Clouds
# ================================================
import argparse
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# ==========================================
# CONFIGURATION CLASS
# ==========================================
class PipelineConfig:
    """
    Holds parameters for the point cloud processing pipeline.
    You can add, change or remove any of the parameters here.
    """
    def __init__(self):
        # Topic settings
        self.topic = '/oakd/points' # Topic containing the pointcloud
        
        # Voxel Downsampling
        self.voxel_size = 0.02
        
        # Normal Estimation
        self.normal_est_k = 15  # Number of neighbors for normal estimation
        
        # Passthrough/Box Filter (Min/Max XYZ)
        self.box_min = np.array([-1.6, -1.4, 0.2]) 
        self.box_max = np.array([ 1.6,  1.4, 4.0]) 

        # Plane RANSAC
        self.floor_dist = 0.02
        self.target_normal = np.array([0.0, 1.0, 0]) # Assuming Y-up for floor
        self.normal_thresh = 0.85
        self.plane_iters = 340  # Number of RANSAC iterations for plane fitting
        self.plane_min_inliers = 150  # Minimum inliers to accept a plane
        self.plane_expected_d = 0.0  # Expected d parameter in plane equation (ax+by+cz+d=0)
        self.plane_d_tolerance = 0.1  # Allowable deviation from expected d
        
        # Cylinder RANSAC
        self.cyl_radius = 0.055
        self.cyl_radius_tolerance = 0.01  # Allowable deviation from expected radius
        self.cyl_iters = 500  # Number of RANSAC iterations
        self.cyl_inlier_thresh = 0.02  # Distance threshold for inliers
        self.min_cylinder_inliers = 20  # Minimum inliers to accept a cylinder
        self.cyl_axis_alignment_thresh = 0.8  # Minimum dot product with [0,0,1] for vertical alignment
        self.cyl_center_x_min = -1.0  # Minimum x-coordinate for cylinder center
        self.cyl_center_x_max = 1.0  # Maximum x-coordinate for cylinder center
        self.cyl_center_y_min = -0.6  # Minimum y-coordinate for cylinder center
        self.cyl_center_y_max = 0.6  # Maximum y-coordinate for cylinder center
        self.cyl_center_z_min = 0.0  # Minimum z-coordinate for cylinder center
        self.cyl_center_z_max = 0.5  # Maximum z-coordinate for cylinder center
        self.min_pts_for_cylinder_ransac = 50  # Minimum points required to attempt cylinder detection
        self.max_cylinders = 3
        self.cyl_visual_height = 0.4  # Height of cylinder for visualization  


# ==========================================
# VISUALIZER CLASS
# ==========================================
class CylinderVisualizer:
    """
    Handles the creation and publishing of RViz MarkerArrays to represent 
    detected cylinders.
    """
    def __init__(self, publisher):
        self.pub_markers = publisher

    def create_cylinder_marker(self, center, radius, rgb, marker_id, frame_id):
        m = Marker()
        m.header.frame_id = frame_id
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        
        m.pose.position.x = float(center[0])
        m.pose.position.y = float(0.0) # Snap to floor level for visualization
        m.pose.position.z = float(center[2])
        
        # Rotate cylinder to stand upright
        m.pose.orientation.x = 0.7071
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 0.7071
        
        m.scale.x = float(radius * 2.0)
        m.scale.y = float(radius * 2.0)
        m.scale.z = float(self.cfg.cyl_visual_height) 
        
        m.color.r = float(rgb[0])
        m.color.g = float(rgb[1])
        m.color.b = float(rgb[2])
        m.color.a = 0.8
        return m

    def publish_viz(self, cylinders, frame_id):
        ma = MarkerArray()
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        ma.markers.append(clear_marker)

        for i, (model, rgb, name) in enumerate(cylinders):
            center, _, radius = model
            marker = self.create_cylinder_marker(center, radius, rgb, 2000 + i, frame_id)
            ma.markers.append(marker)
        
        self.pub_markers.publish(ma)

# ==========================================
# PIPELINE LOGIC 
# ==========================================
class CylinderPipeline:
    def __init__(self, cfg: PipelineConfig):
        self.cfg = cfg
    
    def rgb_to_hsv(self, r, g, b):
        """
        Converts a single RGB point to HSV color space.
        
        :param r: Red component (0.0 - 1.0)
        :param g: Green component (0.0 - 1.0)
        :param b: Blue component (0.0 - 1.0)
        :return: Tuple (h, s, v) where H is [0, 360], S and V are [0, 1]
        """
        mx = max(r, g, b)
        mn = min(r, g, b)
        df = mx - mn
        
        # Calculate Hue
        if mx == mn:
            h = 0
        elif mx == r:
            h = (60 * ((g - b) / df) + 360) % 360
        elif mx == g:
            h = (60 * ((b - r) / df) + 120) % 360
        elif mx == b:
            h = (60 * ((r - g) / df) + 240) % 360
            
        # Calculate Saturation
        s = 0 if mx == 0 else (df / mx)
        
        # Calculate Value
        v = mx
        
        return h, s, v

    def get_neighbors(self, pts, queries, k=15):
        """
        Calculates k-nearest neighbors using a KDTree.
        
        :param pts: The source point cloud (Nx3).
        :param queries: The points for which we want neighbors (Mx3).
        :param k: Number of neighbors to find.
        :return: Indices of neighbors in the 'pts' array.
        """
        if len(pts) < k: return None
        tree = cKDTree(pts)
        _, idxs = tree.query(queries, k=k)
        return idxs

    def box_filter(self, pts, colors):
        """
        Removes points outside the specified XYZ bounding box.
        
        :param pts: Input XYZ array.
        :param colors: Input RGB array.
        :return: Tuple of (filtered_pts, filtered_colors).
        """
        mask = (pts[:, 0] >= self.cfg.box_min[0]) & (pts[:, 0] <= self.cfg.box_max[0]) & \
               (pts[:, 1] >= self.cfg.box_min[1]) & (pts[:, 1] <= self.cfg.box_max[1]) & \
               (pts[:, 2] >= self.cfg.box_min[2]) & (pts[:, 2] <= self.cfg.box_max[2])
        return pts[mask], colors[mask]

    def downsample(self, pts, colors):
        """
        Reduces point cloud density using a voxel grid approach.
        
        Implementation Hint: Convert points to integer coordinates by dividing 
        by voxel_size, then use np.unique to find one point per voxel.
        """
        voxel_coords = np.floor(pts / self.cfg.voxel_size).astype(int)
        unique_voxels, indices = np.unique(voxel_coords, axis=0, return_index=True)
        return pts[indices], colors[indices]

    def estimate_normals(self, pts):
        """
        Estimates a surface normal for every point.
        
        Implementation Hint: 
        1. For each point, find k-neighbors.
        2. Compute the Singular Value Decomposition (SVD), using np.linalg.svd, of the centered neighbors.
        3. The normal is the eigenvector corresponding to the smallest eigenvalue.
        """
        normals = np.zeros_like(pts)
        for i in range(len(pts)):
            neighbors = self.get_neighbors(pts, pts[i:i+1], self.cfg.normal_est_k)
            if neighbors is None or len(neighbors) < 3:
                normals[i] = [0, 0, 1]  # default vertical
                continue
            centered = pts[neighbors] - pts[i]
            u, s, vh = np.linalg.svd(centered)
            normal = vh[-1]  # smallest eigenvalue
            normals[i] = normal
        return normals

    def find_plane_ransac(self, pts):
        """
        Fits a plane model (ax + by + cz + d = 0) to the cloud using RANSAC.
        
        Implementation Hint: 
        1. Sample 3 random points to define a plane.
        2. Calculate the normal and check if it aligns with self.cfg.target_normal.
        3. Count how many points are within self.cfg.floor_dist of the plane.
        4. Return the model with the most inliers.
        """
        best_inliers = np.zeros(len(pts), dtype=bool)
        best_model = None
        for _ in range(self.cfg.plane_iters):
            idx = np.random.choice(len(pts), 3, replace=False)
            p1, p2, p3 = pts[idx]
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm_len = np.linalg.norm(normal)
            if norm_len == 0:
                continue
            normal /= norm_len
            if np.dot(normal, self.cfg.target_normal) < self.cfg.normal_thresh:
                continue
            d = -np.dot(normal, p1)
            if abs(d - self.cfg.plane_expected_d) > self.cfg.plane_d_tolerance:
                continue
            distances = np.abs(np.dot(pts, normal) + d)
            inliers = distances < self.cfg.floor_dist
            if np.sum(inliers) > np.sum(best_inliers):
                best_inliers = inliers
                best_model = (normal, d)
        # Only return if we have enough inliers
        if best_model is not None and np.sum(best_inliers) >= self.cfg.plane_min_inliers:
            return best_model, best_inliers
        return None, np.zeros(len(pts), dtype=bool)

    def find_single_cylinder(self, pts, normals):
        """
        Fits a cylinder model to the remaining points using RANSAC.
        
        Implementation Hint:
        1. Sample 2 points and their normals.
        2. The cylinder axis is roughly the cross product of the two normals.
        3. Check axis alignment with the vertical.
        4. Project points and find distance to the axis; compare to self.cfg.cyl_radius.
        """
        best_inliers = np.zeros(len(pts), dtype=bool)
        best_model = None
        for _ in range(self.cfg.cyl_iters):
            idx = np.random.choice(len(pts), 2, replace=False)
            p1, p2 = pts[idx]
            n1, n2 = normals[idx]
            axis = np.cross(n1, n2)
            axis_len = np.linalg.norm(axis)
            if axis_len == 0:
                continue
            axis /= axis_len
            # Check vertical alignment (assuming z-up)
            if abs(axis[2]) < self.cfg.cyl_axis_alignment_thresh:
                continue
            # Center as midpoint
            center = (p1 + p2) / 2
            # Check center ranges
            if not (self.cfg.cyl_center_x_min <= center[0] <= self.cfg.cyl_center_x_max):
                continue
            if not (self.cfg.cyl_center_y_min <= center[1] <= self.cfg.cyl_center_y_max):
                continue
            if not (self.cfg.cyl_center_z_min <= center[2] <= self.cfg.cyl_center_z_max):
                continue
            # Distances to axis
            vecs = pts - center
            proj = np.outer(np.dot(vecs, axis), axis)
            dists = np.linalg.norm(vecs - proj, axis=1)
            # Inliers where distance close to radius
            inliers = np.abs(dists - self.cfg.cyl_radius) < self.cfg.cyl_inlier_thresh
            num_inliers = np.sum(inliers)
            if num_inliers < self.cfg.min_cylinder_inliers:
                continue
            # Estimate radius from inliers
            estimated_radius = np.median(dists[inliers])
            # Check if estimated radius is within tolerance
            if abs(estimated_radius - self.cfg.cyl_radius) > self.cfg.cyl_radius_tolerance:
                continue
            # Recompute inliers with estimated radius
            inliers = np.abs(dists - estimated_radius) < self.cfg.cyl_inlier_thresh
            num_inliers = np.sum(inliers)
            if num_inliers > np.sum(best_inliers):
                best_inliers = inliers
                best_model = (center, axis, estimated_radius)
        # Only return if we have enough inliers
        if best_model is not None and np.sum(best_inliers) >= self.cfg.min_cylinder_inliers:
            return best_model, best_inliers
        return None, np.zeros(len(pts), dtype=bool)

# ==========================================
# ROS NODE
# ==========================================
class CylinderProcessorNode(Node):
    def __init__(self):
        super().__init__('cylinder_processor_node')
        self.cfg = PipelineConfig()
        self.pipeline = CylinderPipeline(self.cfg)
        
        # Publishers for debugging the pipeline stages in RViz
        self.pub_stage0 = self.create_publisher(PointCloud2, 'pipeline/stage0_box', 10)
        self.pub_stage1 = self.create_publisher(PointCloud2, 'pipeline/stage1_downsample', 10)
        self.pub_stage2 = self.create_publisher(PointCloud2, 'pipeline/stage2_normals', 10)
        self.pub_stage3 = self.create_publisher(PointCloud2, 'pipeline/stage3_candidates', 10)
        self.pub_stage4 = self.create_publisher(PointCloud2, 'pipeline/stage4_cylinder_inliers', 10)
        
        # Marker publisher for the final detection results
        marker_pub = self.create_publisher(MarkerArray, 'viz/detections', 10)
        self.visualizer = CylinderVisualizer(marker_pub)
        # Additional marker publishers for debug visualization of intermediate results
        self.marker_pub_floor = self.create_publisher(MarkerArray, 'viz/floor_plane', 10)
        self.marker_pub_candidates = self.create_publisher(MarkerArray, 'viz/cylinder_candidates', 10)
        self.marker_pub_confirmed = self.create_publisher(MarkerArray, 'viz/cylinder_confirmed', 10)

        self.sub = self.create_subscription(PointCloud2, self.cfg.topic, self.listener_callback, 10)
        
    def numpy_to_pc2_rgb(self, pts, colors, frame_id):
        """
        Converts Nx3 XYZ coordinates and Nx3 RGB color arrays into a ROS 2 PointCloud2 message.
        
        This utility handles the conversion of floating-point spatial data and the packing
        of three 8-bit color channels (R, G, B) into a single 32-bit float field, which is 
        the standard format for RGB point clouds in ROS and RViz.

        :param pts: A numpy array of shape (N, 3) containing [x, y, z] coordinates.
        :param colors: A numpy array of shape (N, 3) containing [r, g, b] values (0.0 to 1.0).
        :param frame_id: The TF frame string (e.g., 'camera_link') for the message header.
        :return: A sensor_msgs/PointCloud2 message ready for publishing.
        """
        msg = PointCloud2()
        msg.header.frame_id, msg.height, msg.width = frame_id, 1, len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 16, True
        msg.row_step = 16 * len(pts)
        c = (np.clip(colors, 0, 1) * 255).astype(np.uint32)
        rgb_packed = (255 << 24) | (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
        data = np.hstack([pts.astype(np.float32), rgb_packed.view(np.float32).reshape(-1, 1)])
        msg.data = data.tobytes()
        return msg
    
    def listener_callback(self, msg):
        """
        Main ROS Callback. Orchestrates the flow from PointCloud2 to Cylinder detection.
        """
        frame_id = msg.header.frame_id
        stride = msg.point_step // 4 
        raw_data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, stride)
        
        # 1. Extraction: Get XYZ points and Filter NaNs
        pts = raw_data[:, :3]
        finite_mask = np.all(np.isfinite(pts), axis=1)
        pts = pts[finite_mask]
        
        # 2. Color Extraction: Decode packed float32 RGB values
        rgb_uint32 = raw_data[finite_mask, 4].view(np.uint32)
        raw_colors = np.vstack([
            ((rgb_uint32 >> 16) & 0xFF) / 255.0, # Red
            ((rgb_uint32 >> 8) & 0xFF) / 255.0,  # Green
            (rgb_uint32 & 0xFF) / 255.0          # Blue
        ]).T

        # TODO: Implement the call sequence:
        pts_box, colors_box = self.pipeline.box_filter(pts, raw_colors)
        self.pub_stage0.publish(self.numpy_to_pc2_rgb(pts_box, colors_box, frame_id))
        
        pts_v, colors_v = self.pipeline.downsample(pts_box, colors_box)
        
        normals = self.pipeline.estimate_normals(pts_v)
        
        plane_model, floor_inliers = self.pipeline.find_plane_ransac(pts_v)
        if plane_model is not None:
            pts_no_floor = pts_v[~floor_inliers]
            colors_no_floor = colors_v[~floor_inliers]
            normals_no_floor = normals[~floor_inliers]
        else:
            pts_no_floor = pts_v
            colors_no_floor = colors_v
            normals_no_floor = normals
        
        self.pub_stage3.publish(self.numpy_to_pc2_rgb(pts_no_floor, colors_no_floor, frame_id))
        
        # Find cylinders
        detected_cylinders = []
        remaining_pts = pts_no_floor
        remaining_normals = normals_no_floor
        remaining_colors = colors_no_floor
        for i in range(self.cfg.max_cylinders):
            if len(remaining_pts) < self.cfg.min_pts_for_cylinder_ransac:
                break
            model, inliers = self.pipeline.find_single_cylinder(remaining_pts, remaining_normals)
            if model is None:
                break
            center, axis, radius = model
            # Assign color, perhaps average color of inliers
            cyl_colors = remaining_colors[inliers]
            avg_color = np.mean(cyl_colors, axis=0) if len(cyl_colors) > 0 else [0.5, 0.5, 0.5]
            detected_cylinders.append((model, avg_color, f"cylinder_{i}"))
            # Remove inliers
            remaining_pts = remaining_pts[~inliers]
            remaining_normals = remaining_normals[~inliers]
            remaining_colors = remaining_colors[~inliers] 
        
        self.visualizer.publish_viz(detected_cylinders, frame_id)

def main():
    rclpy.init()
    node = CylinderProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
