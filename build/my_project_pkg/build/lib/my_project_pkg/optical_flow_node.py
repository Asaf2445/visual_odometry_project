#!/usr/bin/env python3
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .camera_calibrate_params import camera_matrix, distortion_coefficients 
from message_filters import ApproximateTimeSynchronizer, Subscriber
import yaml

# Define the QoS profile to match the expected QoS settings of the publisher


class OpticalFlowVelNode(Node):
    def __init__(self):
        super().__init__('optical_flow_run')
        self.is_filter = self.declare_parameter("filter",value=1).value



        self.tf_broadcaster = TransformBroadcaster(self)
        self.t = TransformStamped()
        self.t.transform.translation.x = 0.0
        self.t.transform.translation.y = 0.0

        self.config = self.load_config()
        self.image_topic = self.config['topics']['image_topic']
        self.imu_topic = self.config['topics']['imu_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.camera_height = self.config['params']['camera_height']

        self.cam_sub = Subscriber(self, CompressedImage, self.image_topic)
        self.imu_subscriber = Subscriber(self, Imu, self.imu_topic )
        self.sync = ApproximateTimeSynchronizer([self.cam_sub, self.imu_subscriber], 10, 0.5, allow_headerless=True)
        self.sync.registerCallback(self.callback_image)
        self.publisher_ = self.create_publisher(Image,self.output_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, "contour_marker_topic", 10)  
        
       

        self.contour_points = []
        self.cv_bridge = CvBridge()

       
        self.feature_params = dict(maxCorners = 100,
                      qualityLevel = 0.1,
                      minDistance = 7,
                      blockSize = 7)

        self.lk_params = dict(winSize = (15, 15),
                 maxLevel =2,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        
        self.xfocal = camera_matrix[0, 0]
        self.yfocal = camera_matrix[1, 1]
        self.Cx = camera_matrix[2, 2]
        self.Cy = camera_matrix[1, 2]
        self.z = 300
        self.frame_count = 0
        self.param_factor = 50/30.24727637
        self.Flag = 0
        self.contour = []
        self.p0 = None
        self.prev_gray = None
        self.point = []
        self.ave_dist_list =np.array([])
        self.counter = 0
        self.ave_vel = 0.0
        self.contour_counter = 0
        self.elapsed_time = None
        self.yaw = 0
        self.deviation_list = []
        self.temp_deviation = 0
        self.previous_timestamp = None
        self.prev_time = None
        self.delta_t = 0
        self.distance = 0
        self.a_x = 0
        self.a_y = 0
        self.position_measure = np.array([[0],[0]])   
        self.rotation_matrix = np.zeros((2,2)) 
        self.a_x_mean = []
        self.a_y_mean = []
        self.msg_counter =0
        self.real_vector = [0 ,0] 
        self.img_vector = [0, 0]
        # Initialize Kalman filter variables]
  # Initial state estimate [position, velocity]
  
        self.X = np.zeros((4,1))
        self.P = np.eye(4)         # Initial covariance matrix
        self.Q = np.array([[100, 0, 0, 0],
                           [0, 100, 0, 0],
                           [0, 0, 1e-1, 0],
                           [0, 0, 0, 1e-1]])*0.0005# Process noise covariance

        self.R= np.array([[1e-4, 0, 0, 0],
                           [0, 1e-4, 0, 0],
                           [0, 0, 1e-4, 0],
                           [0, 0, 0, 1e-4]]) # Measurement noise covariance for good resullts

  # State transition matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  # Measurement matrix
        self.S_k = np.eye((4))
            
    def load_config(self):
        # Default path for the host machine
        default_config_path = '/home/asaf/visual_odometry_project/src/project_bringup/config/config.yaml'
        # Read from environment variable with a fallback to the default
        config_path = os.getenv('CONFIG_PATH', default_config_path)
        with open(config_path, 'r') as file:
            # Load your configuration
            config = yaml.safe_load(file)
        return config      

    def publish_marker(self, step, namespace):
        contour_marker = Marker()
        #contour_marker.header.frame_id = "camera_color_optical_frame"  # Change to your frame
        contour_marker.header.frame_id = "world" 
        contour_marker.type = Marker.LINE_STRIP
        contour_marker.action = Marker.ADD
        contour_marker.scale.x = 0.1  # Line width
        if namespace == "with_filter":
          contour_marker.color.r = 1.0  # Red
        elif namespace == "measurement_only":
           contour_marker.color.b = 1.0 # blue
        else:
           contour_marker.color.g = 1.0 # blue

        contour_marker.color.a = 1.0  # Fully opaque
        contour_marker.ns = f"{namespace}"


        if self.Flag == 0:
          self.point = [0.0, 0.0]
          self.Flag += 1
        else:
         if np.linalg.norm(step) !=0:
            self.point[0] = float(step[0])
            self.point[1] = float(step[1])

            self.t.transform.translation.x =self.point[0]
            self.t.transform.translation.y = self.point[1]
        #self.point = [self.point[0]+step[0], self.point[1]-step[1]]
        self.point_pub = Point(x=self.point[0], y=self.point[1], z=0.0)
        self.contour_points.append(self.point_pub) 
      
        contour_marker.points = self.contour_points
        self.marker_publisher.publish(contour_marker)
        
        
     
    def get_features_dis(self,p0, p1, _st, div):
      static_features = []
      dynamic_features = []
      i, j, k = 0, 0 ,0
      for [x0, y0], [x1, y1] , good_flag in zip(np.float32(p0).reshape(-1, 2), np.float32(p1).reshape(-1, 2), _st):
              precentile_95 = np.percentile(div, 95)
              if good_flag == 1:
                   if   div[k] < precentile_95 :
                     k += 1
                     if i >= len(static_features):
                       static_features.append([])
                     static_features[i].append([x0,y0])
                     static_features[i].append([x1,y1])  
                     x, y = (int(x1)-int(x0))*4, (int(y1)- int(y0))*4
                     cv2.arrowedLine(self.img, (int(x0), int(y0)), (int(x1)+x, int(y1)+y), (0, 255, 0), thickness=2, tipLength=0.4)
                     i += 1
                   else : 
                       k += 1
                       if j>= len(dynamic_features):
                         dynamic_features.append([])
                       dynamic_features[j].append([x0,y0])
                       dynamic_features[j].append([x1,y1])
                       x, y = (int(x1)-int(x0))*4, (int(y1)- int(y0))*4
                       cv2.arrowedLine(self.img, (int(x0), int(y0)), (int(x1)+x, int(y1)+y), (0, 0, 255), thickness=2, tipLength=0.4)
                       j += 1
                       
      
      cv2.polylines(self.img, [np.int32(trajectory) for trajectory in static_features], False, (0, 255, 0), 2)
      cv2.polylines(self.img, [np.int32(trajectory) for trajectory in dynamic_features], False, (0, 0, 255), 2)
      return np.array(static_features)
    
    def calculate_angles_and_deviation(self,p_vector):
      # Calculate the angle of each vector
      angles = np.arctan2(p_vector[:, 1], p_vector[:, 0])
      # Calculate the mean angle
      mean_angle = np.mean(angles)
      
      # Calculate the deviation from the mean angle
      deviations = angles - mean_angle
      
      # Adjust deviations to be within the range [-pi, pi]
      deviations = (deviations + np.pi) % (2 * np.pi) - np.pi
      
      return np.mean(deviations)

    def get_average_velocity(self, static_features):
        global ave_dist
        ave_dist = 0
        if static_features.ndim < 3:
            return None
        u0 = static_features[:, 0, 0]
        v0 = static_features[:, 0, 1]
        u1 = static_features[:, 1, 0]
        v1 = static_features[:, 1, 1]
       

        
        points1_fisheye_img = np.column_stack([u0, v0])

        points2_fisheye_img = np.column_stack((u1, v1))

      
        points1_undistorted_img = cv2.undistortPoints(points1_fisheye_img, camera_matrix, distortion_coefficients, R=None, P=None)
        points2_undistorted_img = cv2.undistortPoints(points2_fisheye_img, camera_matrix, distortion_coefficients, R=None, P=None)
        points1_undistorted_img = np.column_stack([points1_undistorted_img[:, 0, 0], points1_undistorted_img[:, 0, 1]])
        points2_undistorted_img = np.column_stack([points2_undistorted_img[:, 0, 0], points2_undistorted_img[:, 0, 1]])
        points1_undistorted_img[:,0] = points1_undistorted_img[:,0]*camera_matrix[0,0] + camera_matrix[0,2]
        points1_undistorted_img[:,1] = points1_undistorted_img[:,1]*camera_matrix[1,1] + camera_matrix[1,2]

        points2_undistorted_img[:,0] = points2_undistorted_img[:,0]*camera_matrix[0,0] + camera_matrix[0,2]
        points2_undistorted_img[:,1] = points2_undistorted_img[:,1]*camera_matrix[1,1] + camera_matrix[1,2]

        #for fisheye image
        x1 = self.camera_height/self.xfocal*(points1_undistorted_img[:,0]-self.Cx)
        y1 = self.camera_height/self.yfocal*(points1_undistorted_img[:,1]-self.Cy)
        x2 = self.camera_height/self.xfocal*(points2_undistorted_img[:,0]-self.Cx)
        y2 = self.camera_height/self.yfocal*(points2_undistorted_img[:,1]-self.Cy)
        
        #for undistorted image
        # x1 = self.z/self.xfocal*(points1_fisheye_img[:,0]-self.Cx)
        # y1 = self.z/self.yfocal*(points1_fisheye_img[:,1]-self.Cy)
        # x2 = self.z/self.xfocal*(points2_fisheye_img[:,0]-self.Cx)
        # y2 = self.z/self.yfocal*(points2_fisheye_img[:,1]-self.Cy)

# Extract the undistorted points        
        points1 = np.column_stack((x1, y1))
        points2 = np.column_stack((x2, y2))
        
        ave_dist = np.linalg.norm( points2_undistorted_img - points1_undistorted_img, 2, 1)
        
        ave_dist1 = np.linalg.norm(points2- points1, 2, 1)
       
        #vector1 = np.array((float(np.mean(points2_undistorted_img[:,0])), float(np.mean(points2_undistorted_img[:,1]))))
        #vector0 = np.array((float(np.mean(points1_undistorted_img[:,0])), float(np.mean(points1_undistorted_img[:,1])))) 
        # r_vector1 = np.array((x2, y2))
        # r_vector0 = np.array((x1, y1)) 
        
        vector1 = np.array((float(np.mean(u1)), float(np.mean(v1))))
        vector0 = np.array((float(np.mean(u0)), float(np.mean(v0))))
        vector = vector1 - vector0
        p_vector = points2 - points1

        x = self.precentile_fillter(90, p_vector[:,0])
        y = self.precentile_fillter(90, p_vector[:,1])
        min_size = min(x.shape[0], y.shape[0])
        x = x[:min_size]
        y = y[:min_size]
        r_vector = np.column_stack((x, y))
        deviation = self.calculate_angles_and_deviation(r_vector)
        self.is_twist = self.check_vector(r_vector[:,1], r_vector[:,0])
        r_vector = np.array((float(np.mean(r_vector[:,0])), float(np.mean(r_vector[:,1]))))
        # if abs(deviation) > 1e-6:
          
        #  self.R = np.array([[1e-4, 0, 0, 0],
        #                     [0, 1e-4, 0, 0],
        #                      [0, 0, 1e-4, 0],
        #                      [0, 0, 0, 1e-4]])*10**5
        # else:

        #  self.R = np.array([[1e-4, 0, 0, 0],
        #                     [0, 1e-4, 0, 0],
        #                      [0, 0, 1e-4, 0],
        #                      [0, 0, 0, 1e-4]])*10

     
        return ave_dist1, r_vector/1000, vector/np.linalg.norm(vector)*5

    def precentile_fillter(self, precent, vector):
        div = abs(vector-np.mean(vector))
        precent = np.percentile(div, precent)
        indx = div < precent
        vector = vector[indx]

        return vector
    
    def check_vector(self, vector_1, vector_2):
      num_negative_1 = sum(1 for x in vector_1 if x < 0)
      num_positive_1 = len(vector_1) - num_negative_1
      num_negative_2 = sum(1 for x in vector_2 if x < 0)
      num_positive_2 = len(vector_1) - num_negative_2

      if num_negative_1 == 0:
         return 0
      if num_negative_1/(num_negative_1+num_positive_1) > 0.2:
          return 1
      else:
        return 0
      
    def quaternion_to_rotation_matrix(self,q_w, q_x, q_y, q_z):
      n = q_w*q_w + q_x*q_x + q_y*q_y + q_z*q_z
      s = 2.0/n if n != 0 else 0.0
      wx, wy, wz = s*q_w*q_x, s*q_w*q_y, s*q_w*q_z
      xx, xy, xz = s*q_x*q_x, s*q_x*q_y, s*q_x*q_z
      yy, yz, zz = s*q_y*q_y, s*q_y*q_z, s*q_z*q_z

      return np.array([
          [1.0 - (yy + zz), xy - wz, xz + wy],
          [xy + wz, 1.0 - (xx + zz), yz - wx],
          [xz - wy, yz + wx, 1.0 - (xx + yy)]
    ])

    def rotate_vector(self,v, q_w, q_x, q_y, q_z):
    
      rotation_matrix = self.quaternion_to_rotation_matrix(q_w, q_x, q_y, q_z)

      return np.dot(rotation_matrix, v)

# Example usage

    def quaternion_to_euler(self,q_w, q_x, q_y, q_z):
    # Roll (x-axis rotation)
      sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
      cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
      roll = np.arctan2(sinr_cosp, cosr_cosp)

      # Pitch (y-axis rotation)
      sinp = 2 * (q_w * q_y - q_z * q_x)
      pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.sign(sinp) * np.pi / 2

      # Yaw (z-axis rotation)
      siny_cosp = 2 * (q_w * q_z + q_x * q_y)
      cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
      yaw = np.arctan2(siny_cosp, cosy_cosp) - np.pi /2

      return np.rad2deg(roll), np.rad2deg(pitch), -yaw
    
    #def imu_callback(self, imu_msg):
    def callback_image(self, image_msg, imu_msg):
 

     g = np.array([0.0, 0.0, 9.81])  # Gravity vector
     self.A = np.array([[0, self.delta_t,  0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, self.delta_t],
                        [0, 0, 0, 1]])

     self.B = np.array([[0.5*self.delta_t**2, 0],
                        [self.delta_t, 0 ],
                        [0, 0.5*self.delta_t**2],
                        [0, self.delta_t]])     

     

     if self.prev_time is not None:
          current_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9  # Convert nanoseconds to seconds
          self.delta_t = current_time - self.prev_time



     self.prev_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9  # Convert nanoseconds to seconds
     self.quaternion = imu_msg.orientation
     q = (self.quaternion.w, self.quaternion.x, self.quaternion.y, self.quaternion.z)
     g_body = self.rotate_vector(g, q[0], q[1], q[2], q[3])
     roll, pitch, self.yaw = self.quaternion_to_euler(self.quaternion.w, self.quaternion.x, self.quaternion.y, self.quaternion.z)

     self.rotation_matrix = np.array([[np.cos(self.yaw), -np.sin(self.yaw)],
                                     [np.sin(self.yaw), np.cos(self.yaw)]])

     self.t.header.stamp = self.get_clock().now().to_msg()
     self.t.header.frame_id = 'world'
     self.t.child_frame_id = 'odom'
     self.t.transform.rotation.x = self.quaternion.x
     self.t.transform.rotation.y = self.quaternion.y
     self.t.transform.rotation.z = -self.quaternion.z 
     self.t.transform.rotation.w = self.quaternion.w
     self.tf_broadcaster.sendTransform(self.t)
     #g = self.rotate_vector(g, self.quaternion.w, self.quaternion.x, self.quaternion.y, self.quaternion.z)

     if len(self.a_x_mean) < 30 :
        self.a_x_mean.append(imu_msg.linear_acceleration.x)
        self.a_y_mean.append(imu_msg.linear_acceleration.y)
        self.a_x, self.a_y, a_z = 0, 0, 0
     else:
        x_mean = sum(self.a_x_mean)/len(self.a_x_mean)
        y_mean = sum(self.a_y_mean)/len(self.a_y_mean)
        self.a_x = imu_msg.linear_acceleration.x - g_body[1] 
        self.a_y = imu_msg.linear_acceleration.y + g_body[0] 




     u = np.array([[self.a_y], [self.a_x]]) 
     
     self.X = np.dot(self.A, self.X) + self.B @ u
     self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
    #  print(f"\n\n g_vector : {g_body} \n\n a_vector : {[imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z]} \n\n a-g : {[u.reshape((1,2)), imu_msg.linear_acceleration.z-g_body[2]]}" )
     #self.P = np.dot(np.dot(self.A, self.P), self.A.T) + np.dot(self.B, self.B.T)*0.01**2


    #def callback_image(self, image_msg):
     self.frame = self.cv_bridge.compressed_imgmsg_to_cv2(image_msg,'bgr8')
     self.frame = self.frame[0:250, 500:1100]

    #  h, w = self.frame.shape[:2]
    #  newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
            

    #         # crop the image
    #  x, y, w, h = roi
    #  dst = cv2.undistort(self.frame, camera_matrix, distortion_coefficients, None, camera_matrix)
            
    #  self.frame = dst[y:y + h, x:x + w]

    
     frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY )
     self.img = self.frame.copy()
   

     if self.p0 is not None:
           
           img0, img1 = self.prev_gray, frame_gray
           p1, st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, self.p0, None, **self.lk_params)
           
           if p1 is not None:
              good_new = p1[st==1]
              good_old = self.p0[st==1]
           distance = np.linalg.norm(np.float32(good_new).reshape(-1, 2) - np.float32(good_old).reshape(-1, 2), 2, 1)
           div = abs(distance - np.mean(distance))
           static_features = self.get_features_dis(good_old, good_new, st, div)
           if len(static_features) <= 5:
              pass
           else:
            ave_dist, self.real_vector, self.img_vector = self.get_average_velocity(static_features)   
          #  correct_measure = np.dot(self.rotation_matrix, np.array([[self.real_vector[1]], [self.real_vector[0]]]))
           self.position_measure = self.position_measure.astype(np.float64)
          #  self.position_measure += correct_measure


           #Kalman filter
           z_k = np.array([[self.real_vector[0]],[self.real_vector[0]/self.delta_t], [self.real_vector[1]], [self.real_vector[1]/self.delta_t]])
           y_k = z_k - np.dot(self.H, self.X)
           mahalanobis_distance = np.sqrt(np.dot(np.dot(y_k.T, np.linalg.inv(self.S_k)), y_k))
           if mahalanobis_distance > 3:
              self.R = np.array([[1e-4, 0, 0, 0],
                                 [0, 1e-4, 0, 0],
                                 [0, 0, 1e-4, 0],
                                 [0, 0, 0, 1e-4]])*10**5
           else:
              self.R = np.array([[1e-4, 0, 0, 0],
                                 [0, 1e-4, 0, 0],
                                 [0, 0, 1e-4, 0],
                                 [0, 0, 0, 1e-4]])
           self.S_k = self.R + np.dot(np.dot(self.H, self.P), self.H.T)
           K_k = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(self.S_k))
           if self.is_filter == 1:  
            self.X = self.X + np.dot(K_k, y_k) 
           self.P = np.dot((np.eye(4) - np.dot(K_k, self.H)), self.P)

           if self.is_filter == True:
            self.position_measure += np.dot(self.rotation_matrix, np.array([[self.X[0,0]], [self.X[2,0]]]))
            self.publish_marker(self.position_measure, "with_filter")
            # self.publish_marker(self.position_measure, "with_filter")

           elif self.is_filter == 0:
            self.position_measure += np.dot(self.rotation_matrix, np.array([[self.real_vector[0]*0], [self.real_vector[1]]]))
            self.publish_marker(self.position_measure, "measurement_only")
           else: 
            self.position_measure += np.dot(self.rotation_matrix, np.array([[self.X[0,0]], [self.X[2,0]]]))
            self.publish_marker(self.position_measure, "model_only")


           self.distance = np.linalg.norm(self.X[:2,0])
         
          #  if self.counter == 0:
          #    self.ave_dist_list = np.mean(ave_dist)
          #    self.counter += 1
          #  else:
          #    self.ave_dist_list = np.vstack((self.ave_dist_list, np.mean(ave_dist)))
          #    self.counter += 1

          #  if np.mean(ave_dist) == None:
          #   return 


           ave_vel = np.linalg.norm(self.X[2:4,0]) # [mm] to [M] 
             
              
           if ave_vel < 0.1:
                cv2.putText(self.img, "Velocity: 0.0 [M/s]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
           elif self.img_vector is not None:
                cv2.putText(self.img, f"Velocity: {ave_vel: .1f} [M/s]", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
           cv2.arrowedLine(self.img, (int(self.frame.shape[1]-self.frame.shape[1]/10), int(self.frame.shape[0]/10)), (int(self.frame.shape[1]-self.frame.shape[1]/10)+(int(self.img_vector[0]))*-6, int(self.frame.shape[0]/10)+(int(self.img_vector[1]))*-6), (255, 0, 0), thickness=4, tipLength=1)
     self.prev_gray = frame_gray
     mask = np.zeros_like(frame_gray)
     mask[:] = 255
    
     self.p0 = cv2.goodFeaturesToTrack(frame_gray,mask = mask, **self.feature_params )
     img_msg = self.cv_bridge.cv2_to_imgmsg(self.img, 'bgr8')
     self.publisher_.publish(img_msg)
       #self.get_logger().info('Publishing proccess image...')
     

def main(args=None):
    rclpy.init(args=args)
    optical_flow_node = OpticalFlowVelNode()
    while rclpy.ok():
        rclpy.spin_once(optical_flow_node, timeout_sec=0.1)  # Use spin_once with a timeout
    #rclpy.spin(optical_flow_node)
    optical_flow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    