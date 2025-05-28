import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import pybullet as p
import numpy as np
import transforms3d
import pybullet_data
import time
import matplotlib.pyplot as plt
import cv2

class LaserScanPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)

    def publish_lidar_scan(self, lidar_data):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = -3.14
        msg.angle_max = 3.14
        msg.angle_increment = 6.28 / len(lidar_data)
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = 2.0
        msg.ranges = lidar_data
        msg.intensities = []
        self.publisher_.publish(msg)

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.last_velocity = None
        self.last_time = self.get_clock().now()

    def publish_imu_data(self, deepracer_id):
        current_time = self.get_clock().now()
        delta_time_sec = (current_time - self.last_time).nanoseconds / 1e9
        orientation_quat, orientation_euler, angular_velocity, linear_velocity = self.get_imu_data(deepracer_id)
        if self.last_velocity is not None and delta_time_sec > 0:
            linear_acceleration = [(lv - lv_last) / delta_time_sec for lv, lv_last in zip(linear_velocity, self.last_velocity)]
        else:
            linear_acceleration = [0.0, 0.0, 0.0]
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = orientation_quat
        imu_msg.orientation_covariance = [0.0]*9
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = angular_velocity
        imu_msg.angular_velocity_covariance = [0.0]*9
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = linear_acceleration
        imu_msg.linear_acceleration.z += 9.81
        imu_msg.linear_acceleration_covariance = [0.0]*9
        self.publisher_.publish(imu_msg)
        self.last_velocity = linear_velocity
        self.last_time = current_time
        
    def get_imu_data(self, deepracer_id):
        position, orientation_quat = p.getBasePositionAndOrientation(deepracer_id)
        orientation_euler = p.getEulerFromQuaternion(orientation_quat)
        linear_velocity, angular_velocity = p.getBaseVelocity(deepracer_id)
        return orientation_quat, orientation_euler, angular_velocity, linear_velocity

class ServoControlSubscriber(Node):
    def __init__(self, deepracer_id):
        super().__init__('servo_control_subscriber')
        self.subscription = self.create_subscription(ServoCtrlMsg, 'cmd_servo_control', self.listener_callback, 10)
        self.deepracer_id = deepracer_id
        self.wheel_joints = self.find_wheel_joints()

    def find_wheel_joints(self):
        wheel_joints = {}
        joint_names = ['left_rear_wheel_joint', 'right_rear_wheel_joint', 'left_front_wheel_joint', 'right_front_wheel_joint', 'left_steering_hinge_joint', 'right_steering_hinge_joint']
        for joint_index in range(p.getNumJoints(self.deepracer_id)):
            joint_info = p.getJointInfo(self.deepracer_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name in joint_names:
                wheel_joints[joint_name] = joint_index
        return wheel_joints

    def control_deepracer(self, throttle_value, steering_angle):
        # Apply differential drive logic to rear wheels
        p.setJointMotorControl2(self.deepracer_id,
                                self.wheel_joints['left_rear_wheel_joint'],
                                p.VELOCITY_CONTROL,
                                targetVelocity=throttle_value)
        p.setJointMotorControl2(self.deepracer_id,
                                self.wheel_joints['right_rear_wheel_joint'],
                                p.VELOCITY_CONTROL,
                                targetVelocity=throttle_value) 
        # Steering for front wheels
        steering_angle_radians = np.radians(steering_angle)  # Convert degrees to radians if needed
        p.setJointMotorControl2(self.deepracer_id,
                                self.wheel_joints['left_steering_hinge_joint'],
                                p.POSITION_CONTROL,
                                targetPosition=steering_angle_radians)
        p.setJointMotorControl2(self.deepracer_id,
                                self.wheel_joints['right_steering_hinge_joint'],
                                p.POSITION_CONTROL,
                                targetPosition=steering_angle_radians)

    def listener_callback(self, msg):
        throttle_value = msg.throttle * 15  # Scale throttle value
        steering_angle = msg.angle * 15    # Scale angle for steering
        self.control_deepracer(throttle_value, steering_angle)


def setup_environment():
    physicsClient = p.connect(p.GUI) #p.DIRECT
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

def load_urdf(urdf_path, basePosition, orientation, useFixedBase=False):
    quaternion_orientation = p.getQuaternionFromEuler(orientation)
    object_id = p.loadURDF(urdf_path, basePosition=basePosition, baseOrientation=quaternion_orientation, useFixedBase=useFixedBase)
    return object_id

def load_static_urdf(urdf_path, position, orientation):
    quaternion_orientation = p.getQuaternionFromEuler(orientation)
    
    object_id = p.loadURDF(urdf_path, 
                           basePosition=position, 
                           baseOrientation=quaternion_orientation,
                           useFixedBase=True)
    
    return object_id

def compute_transformation(position, orientation_rpy):
    rotation_matrix = transforms3d.euler.euler2mat(*orientation_rpy, axes='sxyz')
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = position
    return T

def capture_camera_image(deepracer_id, T_camera_final, img_w=640, img_h=480):
    base_pos, base_orn = p.getBasePositionAndOrientation(deepracer_id)
    base_orn_euler = p.getEulerFromQuaternion(base_orn)
    R_base = transforms3d.euler.euler2mat(*base_orn_euler)
    T_base = np.eye(4)
    T_base[:3, :3] = R_base
    T_base[:3, 3] = base_pos

    T_camera_global = np.dot(T_base, T_camera_final)

    camera_img_data = capture_image(T_camera_global[:3, 3], T_camera_global[:3, 3] + np.dot(T_camera_global[:3, :3], np.array([1, 0, 0])), img_w, img_h)
    
    camera_img_rgb = np.array(camera_img_data[2]).reshape((img_h, img_w, 4))[:, :, :3]
    return camera_img_rgb

def capture_image(camera_pos, camera_target_pos, img_w, img_h):
    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_pos.tolist(), cameraTargetPosition=camera_target_pos.tolist(), cameraUpVector=[0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(fov=50, aspect=float(img_w)/img_h, nearVal=0.02, farVal=3.5)
    img = p.getCameraImage(img_w, img_h, viewMatrix=view_matrix, projectionMatrix=projection_matrix, shadow=False, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return img

def display_image(camera_img):
    plt.figure(figsize=(6, 6))
    plt.imshow(cv2.cvtColor(camera_img, cv2.COLOR_BGR2RGB))
    plt.title('Camera View')
    plt.axis('off')
    plt.pause(0.001)
    plt.show()


## To visualize the lidar data
# def simulate_and_visualize_lidar(deepracer_id, num_rays=60, lidar_range=2):
#     base_position, base_orientation = p.getBasePositionAndOrientation(deepracer_id)
#     base_position = np.array(base_position)
#     base_orientation_euler = p.getEulerFromQuaternion(base_orientation)

#     laser_rel_position = np.array([0.02913, 0, 0.16145])
#     laser_rel_orientation_rpy = np.array([0, 0, np.pi])

#     T_base = compute_transformation(base_position, base_orientation_euler)
#     T_laser_rel = compute_transformation(laser_rel_position, laser_rel_orientation_rpy)

#     T_laser_global = np.dot(T_base, T_laser_rel)

#     laser_global_position = T_laser_global[:3, 3]
#     laser_global_orientation = transforms3d.euler.mat2euler(T_laser_global[:3, :3], axes='sxyz')

#     angles = np.linspace(-np.pi, np.pi, num_rays)
#     direction_vectors = np.array([np.cos(angles), np.sin(angles), np.zeros_like(angles)]).T
#     laser_rotation_matrix = transforms3d.euler.euler2mat(*laser_global_orientation, axes='sxyz')
#     direction_vectors = direction_vectors.dot(laser_rotation_matrix)

#     start_points = np.tile(laser_global_position, (num_rays, 1))
#     end_points = start_points + direction_vectors * lidar_range

#     ray_results = p.rayTestBatch(start_points.tolist(), end_points.tolist())

#     new_ray_ids = []
#     for i, (start, end, result) in enumerate(zip(start_points, end_points, ray_results)):
#         hit_fraction = result[2]
#         hit_point = start + hit_fraction * (end - start)

#         color = [1, 0, 0] if hit_fraction < 1.0 else [0, 1, 0]
#         ray_id = p.addUserDebugLine(start, hit_point if hit_fraction < 1.0 else end, color, 1, lifeTime=1)
#         new_ray_ids.append(ray_id)

#     return new_ray_ids
    
## To get raw data of lidar 
def simulate_and_visualize_lidar(deepracer_id, num_rays=120, lidar_range=2):
    base_position, base_orientation = p.getBasePositionAndOrientation(deepracer_id)
    base_position = np.array(base_position)
    base_orientation_euler = p.getEulerFromQuaternion(base_orientation)

    T_base = compute_transformation(base_position, base_orientation_euler)
    laser_rel_position = np.array([0.02913, 0, 0.16145])  
    laser_rel_orientation_rpy = np.array([0, 0, np.pi]) 

    T_laser_rel = compute_transformation(laser_rel_position, laser_rel_orientation_rpy)
    T_laser_global = np.dot(T_base, T_laser_rel)

    laser_global_position = T_laser_global[:3, 3]
    laser_global_orientation = transforms3d.euler.mat2euler(T_laser_global[:3, :3], axes='sxyz')

    angles = np.linspace(-np.pi, np.pi, num_rays)
    direction_vectors = np.array([np.cos(angles), np.sin(angles), np.zeros_like(angles)]).T
    laser_rotation_matrix = transforms3d.euler.euler2mat(*laser_global_orientation, axes='sxyz')
    direction_vectors = direction_vectors.dot(laser_rotation_matrix)

    start_points = np.tile(laser_global_position, (num_rays, 1))
    end_points = start_points + direction_vectors * lidar_range

    ray_results = p.rayTestBatch(start_points.tolist(), end_points.tolist())

    lidar_data = [result[2] * lidar_range for result in ray_results]

    return lidar_data

def main():
    rclpy.init(args=None)
    laser_scan_publisher = LaserScanPublisher()
    imu_publisher = ImuPublisher()
    setup_environment()

    deepracer_id = load_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/deepracer_description/meshes/1deepracer_front_facing_camera_and_lidar_urdf.urdf", [4.5, 0, 0.1], [0,0,-(1/2)*np.pi])
    # object_id = load_static_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/pybullet-URDF-models/urdf_models/models/blue_moon/model.urdf", [4.0, 0, 0.1], [0, 0, 0.1])

    road_from_Qi_gao = load_static_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/Models_downloaded_from_free3D/URDF_file/road_from_Qi_gao.urdf", [0,0,0], [np.pi/2,0,0])

    tree1 = load_static_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/Models_downloaded_from_free3D/URDF_file/tree.urdf", [-1,0.6,0], [np.pi/2, 0, 0])
    tree2 = load_static_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/Models_downloaded_from_free3D/URDF_file/tree.urdf", [0.6,1,0], [np.pi/2, 0, 0])
    tree3 = load_static_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/Models_downloaded_from_free3D/URDF_file/tree.urdf", [0.6,-1,0], [np.pi/2, 0, 0])
    tree4 = load_static_urdf("/home/do-gon/ros2_foxy_deepracer_ws/src/Models_downloaded_from_free3D/URDF_file/tree.urdf", [-1,-0.6,0], [np.pi/2, 0, 0])

    servo_control_subscriber = ServoControlSubscriber(deepracer_id)

    T_zed_final = compute_transformation([0.091711, 0, 0.080023], [0, 0.2618, 0])
    T_camera_final = np.dot(T_zed_final, compute_transformation([0.044755, 0, 0.04], [0, 0, 0]))

    prev_ray_ids = []

    for step in range(100000):

        ## To visualize the LIDAR
        # if step % 200 == 0:
        #     for ray_id in prev_ray_ids:
        #         p.removeUserDebugItem(ray_id)
        #     prev_ray_ids = simulate_and_visualize_lidar(deepracer_id)
        
        ## To get raw data from LIDAR
        if step % 200 == 0:
            lidar_data = simulate_and_visualize_lidar(deepracer_id)
            laser_scan_publisher.publish_lidar_scan(lidar_data)
            camera_img = capture_camera_image(deepracer_id, T_camera_final)

        imu_publisher.publish_imu_data(deepracer_id)
            
        p.stepSimulation()
        rclpy.spin_once(servo_control_subscriber, timeout_sec=0)
        time.sleep(1./240.)

    display_image(camera_img)

    p.disconnect()
    laser_scan_publisher.destroy_node()
    imu_publisher.destroy_node()
    servo_control_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
