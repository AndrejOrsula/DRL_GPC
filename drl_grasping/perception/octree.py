from drl_grasping.utils import conversions
from geometry_msgs.msg import Transform
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from threading import Thread
from typing import List, Tuple
import numpy as np
import ocnn
import open3d
import rclpy
import tf2_ros
import torch

class OctreeCreator(Node):
    def __init__(self,
                 robot_frame_id: str = "world",
                 min_bound: Tuple[float, float, float] = (-1.0, -1.0, -1.0),
                 max_bound: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                 normals_radius: float = 0.05,
                 normals_max_nn: int = 10,
                 include_color: bool = False,
                 depth: int = 4,
                 full_depth: int = 2,
                 node_dis: bool = True,
                 node_feature: bool = False,
                 split_label: bool = False,
                 adaptive: bool = False,
                 adp_depth: int = 4,
                 th_normal: float = 0.1,
                 th_distance: float = 2.0,
                 extrapolate: bool = False,
                 save_pts: bool = False,
                 key2xyz: bool = False,
                 use_sim_time: bool = True,
                 debug_draw: bool = False,
                 debug_write_octree: bool = False,
                 node_name: str = 'drl_grasping_octree_creator'):

        # Initialise node
        try:
            rclpy.init()
        except:
            if not rclpy.ok():
                import sys
                sys.exit("ROS 2 could not be initialised")
        Node.__init__(self, node_name)
        self.set_parameters([Parameter('use_sim_time',
                                       type_=Parameter.Type.BOOL,
                                       value=use_sim_time)])

        # Create tf2 buffer and listener for transform lookup
        self.__tf2_buffer = tf2_ros.Buffer()
        self.__tf2_listener = tf2_ros.TransformListener(buffer=self.__tf2_buffer,
                                                        node=self)

        # Parameters
        self._robot_frame_id = robot_frame_id
        self._min_bound = min_bound
        self._max_bound = max_bound
        self._normals_radius = normals_radius
        self._normals_max_nn = normals_max_nn
        self._include_color = include_color
        self._depth = depth
        self._full_depth = full_depth
        self._debug_draw = debug_draw
        self._debug_write_octree = debug_write_octree

        # Create a converter between points and octree
        self._points_to_octree = ocnn.Points2Octree(depth=depth,
                                                    full_depth=full_depth,
                                                    node_dis=node_dis,
                                                    node_feature=node_feature,
                                                    split_label=split_label,
                                                    adaptive=adaptive,
                                                    adp_depth=adp_depth,
                                                    th_normal=th_normal,
                                                    th_distance=th_distance,
                                                    extrapolate=extrapolate,
                                                    save_pts=save_pts,
                                                    key2xyz=key2xyz,
                                                    bb_min=min_bound,
                                                    bb_max=max_bound)

        # Spin executor in another thread
        self._executor = MultiThreadedExecutor(2)
        self._executor.add_node(self)
        self._executor.add_node(self.__tf2_listener.node)
        self._executor_thread = Thread(target=self._executor.spin,
                                       args=(),
                                       daemon=True)
        self._executor_thread.start()

    def __call__(self, ros_point_cloud2: PointCloud2) -> torch.Tensor:

        # Convert to Open3D PointCloud
        open3d_point_cloud = conversions.pointcloud2_to_open3d(
            ros_point_cloud2=ros_point_cloud2)

        # Preprocess point cloud (transform to robot frame, crop to workspace and estimate normals)
        open3d_point_cloud = self.preprocess_point_cloud(
            open3d_point_cloud=open3d_point_cloud,
            camera_frame_id=ros_point_cloud2.header.frame_id,
            robot_frame_id=self._robot_frame_id,
            min_bound=self._min_bound,
            max_bound=self._max_bound,
            normals_radius=self._normals_radius,
            normals_max_nn=self._normals_max_nn
        )

        # Draw if needed
        if self._debug_draw:
            open3d.visualization.draw_geometries([open3d_point_cloud,
                                                  open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2,
                                                                                                       origin=[0.0, 0.0, 0.0])],
                                                 point_show_normal=True)

        # Construct octree from such point cloud
        octree = self.construct_octree(open3d_point_cloud,
                                     include_color=self._include_color)

        # Write if needed
        if self._debug_write_octree:
            ocnn.write_octree(octree, 'octree.octree')

        return octree

    def preprocess_point_cloud(self, open3d_point_cloud: open3d.geometry.PointCloud,
                               camera_frame_id: str,
                               robot_frame_id: str,
                               min_bound: List[float],
                               max_bound: List[float],
                               normals_radius: float,
                               normals_max_nn: int) -> open3d.geometry.PointCloud:

        # Check if any points remain in the area after cropping
        if not open3d_point_cloud.has_points():
            print("Point cloud has no points")
            return open3d_point_cloud

        # Get transformation from camera to robot and use it to transform point
        # cloud into robot's base coordinate frame
        transform = self.lookup_transform_sync(target_frame=robot_frame_id,
                                               source_frame=camera_frame_id)
        transform_mat = conversions.transform_to_matrix(transform=transform)
        open3d_point_cloud = open3d_point_cloud.transform(transform_mat)

        # Crop point cloud to include only the workspace
        open3d_point_cloud = open3d_point_cloud.crop(bounding_box=open3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound,
                                                                                                         max_bound=max_bound))

        # Check if any points remain in the area after cropping
        if not open3d_point_cloud.has_points():
            print("Point cloud has no points after cropping to the workspace volume")
            return open3d_point_cloud

        # Estimate normal vector for each cloud point and orient these towards the camera
        open3d_point_cloud.estimate_normals(search_param=open3d.geometry.KDTreeSearchParamHybrid(radius=normals_radius,
                                                                                                 max_nn=normals_max_nn),
                                            fast_normal_computation=True)

        open3d_point_cloud.orient_normals_towards_camera_location(
            camera_location=transform_mat[0:3, 3])

        return open3d_point_cloud

    def construct_octree(self, open3d_point_cloud: open3d.geometry.PointCloud, include_color: bool) -> torch.Tensor:

        # In case the point cloud has no points, add a single point
        # This is a workaround because I was not able to create an empty octree without getting a segfault
        # TODO: Figure out a better way of making an empty octree (it does not occur if setup correctly, so probably not worth it)
        if not open3d_point_cloud.has_points():
            open3d_point_cloud.points.append([(self._min_bound[0] + self._max_bound[0])/2,
                                              (self._min_bound[1] + self._max_bound[1])/2,
                                              (self._min_bound[2] + self._max_bound[2])/2])
            open3d_point_cloud.normals.append([0.0, 0.0, 0.0])
            open3d_point_cloud.colors.append([0.0, 0.0, 0.0])

        # Convert open3d point cloud into octree points
        octree_points = conversions.open3d_point_cloud_to_octree_points(
            open3d_point_cloud, include_color)

        # Convert octree points into 1D Tensor (via ndarray)
        # Note: Copy of points here is necessary as ndarray would otherwise be immutable
        octree_points_ndarray = np.frombuffer(np.copy(octree_points.buffer()),
                                              np.uint8)
        octree_points_tensor = torch.from_numpy(octree_points_ndarray)

        # Finally, create an octree from the points
        return self._points_to_octree(octree_points_tensor)

    def lookup_transform_sync(self,
                              target_frame: str,
                              source_frame: str) -> Transform:

        while rclpy.ok():
            if self.__tf2_buffer.can_transform(target_frame=target_frame,
                                               source_frame=source_frame,
                                               time=rclpy.time.Time(),
                                               timeout=rclpy.time.Duration(seconds=1,
                                                                           nanoseconds=0)):
                transform_stamped = self.__tf2_buffer.lookup_transform(target_frame=target_frame,
                                                                       source_frame=source_frame,
                                                                       time=rclpy.time.Time())
                return transform_stamped.transform

            print(f'Lookup of transform from "{source_frame}"'
                  f' to "{target_frame}" failed, retrying...')
