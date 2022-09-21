#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from matplotlib import pyplot as plt
import copy
import ros_numpy
from geometry_msgs.msg import Point
import time
import tf
from std_msgs.msg import Bool
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Empty
import math
# from cv_bridge import CvBridge
from core import CvBridge


class CONTROLLER:
	def __init__(self):
		rospy.loginfo('INIT')
		self.rate = rospy.Rate(10)
		self.current_position = Point()
		self.current_orientation = Quaternion()
		self.position_publisher = rospy.Publisher('/red/position_hold/trajectory',MultiDOFJointTrajectoryPoint,queue_size=10)
		rospy.Subscriber('/red/odometry',Odometry, self.get_pose)
		time.sleep(0.5)

	def get_pose(self,data):
		data = data.pose.pose
		self.current_position.x = data.position.x
		self.current_position.y = data.position.y
		self.current_position.z = data.position.z
		self.current_orientation.x = data.orientation.x
		self.current_orientation.y = data.orientation.y
		self.current_orientation.w = data.orientation.w
		self.current_orientation.z = data.orientation.z
		self.current_pitch, self.current_roll, self.yaw = self.get_euler_angles()
		self.compute_transformation_matrix()
		# positive yaw is with respect to x-axis in gazebo

	def get_euler_angles(self):
		x, y, z, w = self.current_orientation.x , self.current_orientation.y, self.current_orientation.z, self.current_orientation.w
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t3, t4)
		return roll, pitch, yaw

	def get_quaternion_from_euler(self,yaw):
		qx = np.sin(self.current_roll/2) * np.cos(self.current_pitch/2) * np.cos(yaw/2) - np.cos(self.current_roll/2) * np.sin(self.current_pitch/2) * np.sin(yaw/2)
		qy = np.cos(self.current_roll/2) * np.sin(self.current_pitch/2) * np.cos(yaw/2) + np.sin(self.current_roll/2) * np.cos(self.current_pitch/2) * np.sin(yaw/2)
		qz = np.cos(self.current_roll/2) * np.cos(self.current_pitch/2) * np.sin(yaw/2) - np.sin(self.current_roll/2) * np.sin(self.current_pitch/2) * np.cos(yaw/2)
		qw = np.cos(self.current_roll/2) * np.cos(self.current_pitch/2) * np.cos(yaw/2) + np.sin(self.current_roll/2) * np.sin(self.current_pitch/2) * np.sin(yaw/2)
		return [qx, qy, qz, qw]

	def compute_transformation_matrix(self): # converts coordinates in the drones frame to the global frame 
		self.transformation_matrix = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],[math.sin(self.yaw), math.cos(self.yaw)]])

	def transform_coordinates(self, coordinates): # pass coordinates as [x,y]
		return np.matmul(self.transformation_matrix,np.array([coordinates[0], coordinates[1]]))

	def waypoint(self, x, y, yaw , z = 2.5):
		transforms = Transform()
		velocities = Twist()
		accelerations = Twist()
		transforms.translation.x = x
		transforms.translation.y = y
		transforms.translation.z = z
		q = self.get_quaternion_from_euler(yaw)
		transforms.rotation.x = q[0]
		transforms.rotation.y = q[1]
		transforms.rotation.z = q[2]
		transforms.rotation.w = q[3]
		point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(1))
		self.position_publisher.publish(point)
		self.rate.sleep()
        
class detector:
    def __init__(self):
        self.count = [False]
        self.camera0 = rospy.Subscriber("/red/camera/color/image_raw", Image, self.frontCallback)
        time.sleep(2)
        rospy.Subscriber("/red/camera/depth_registered/points", PointCloud2, self.cam_callback)
        self.listener = tf.TransformListener()
        time.sleep(1)
        self.pub_point = rospy.Publisher('/red/tag_position_reconstructed' , Point, queue_size = 10)
        self.pub_image = rospy.Publisher('/red/tag_image_annotated' , Image, queue_size =10)

        self.cX = 0
        self.cY = 0

        queue_size = 100
        self.xq = np.arange(queue_size)
        self.yq = np.arange(queue_size)
        self.zq = np.arange(queue_size)
        self.yawq = np.arange(queue_size)
        self.bridge = CvBridge()
        
        time.sleep(2)
        

    def frontCallback(self,data):
        self.frontImage = ros_numpy.numpify(data)
        # self.cX, self.cY, self.tl, self.tr, self.br, self.detected = detect_ar_tag(self.frontImage)
        self.detected, self.ct = cv_out(self.frontImage)
        if self.detected and len(self.count) < 20:
            self.count.append(True)
        if not self.detected and len(self.count) < 20:
            self.count.append(False)
        if self.detected and not len(self.count) < 20:
            self.count = np.array(self.count)
            temp = np.copy(self.count)
            self.count[0:-1] = np.copy(temp[1:20])
            self.count[19] = True
        if not self.detected and not len(self.count) < 20:
            self.count = np.array(self.count)
            temp = np.copy(self.count)
            self.count[0:-1] = np.copy(temp[1:20])
            self.count[19] = False

        ######### TO DISPLAY
        # if self.detected:
        #     ct = self.ct[0]
        #     w = cv2.drawContours(self.frontImage.copy(), ct, -1, (0, 255, 0), 5)
        #     cv2.imshow('whatever', w)
        #     cv2.waitKey(1)
        

    def pixel_to_localxyz(self,h,w):
        self.last_data_arr = self.pc_arr
        xp,yp,zp = self.last_data_arr['x'][w, h], self.last_data_arr['y'][w, h], self.last_data_arr['z'][w, h]
        return np.array([xp,yp,zp])
    
    def cam_callback(self, value):
        self.pc_arr = ros_numpy.numpify(value)
        # self.xyzyaw_reqd()

    def cross_product(self,u,v):
        cross = np.array([u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]) #cross product
        mag = np.sqrt(cross[0]**2+cross[2]**2)
        if mag > 0:
            cross = cross/mag
            return cross
        else:
            return cross
    
    def data_point(self):
        self.last_data_arr = self.pc_arr.copy()
        self.last_data_trans, self.last_data_rot = self.listener.lookupTransform("/map", "out", rospy.Time(0))
        self.euler = tf.transformations.euler_from_quaternion(self.last_data_rot)
        self.last_data_rm = tf.transformations.euler_matrix(self.euler[0],self.euler[1],self.euler[2])

    def data_global(self,local):
        xt,yt,zt = local
        tx,ty,tz = np.dot(self.last_data_rm,np.array([xt,yt,zt, 1]))[0:3]+np.array(self.last_data_trans)
        return np.array([tx, ty, tz])

    def __call__(self):
        if len(self.count) > 5:
            if len(np.argwhere(self.count)) > 5 and self.detected:
                self.data_point()
                ct = np.array(self.ct[0])
                h = ct[:, 0, 1]
                w = ct[:, 0, 0]
                x, y, z = self.data_global(self.pixel_to_localxyz(w, h))
                l1 = [x[1] - x[0], y[1] - y[0], z[1] - z[0]]
                l2 = [x[1] - x[2], y[1] - y[2], z[1] - z[2]]
                ut_vector = self.cross_product(l1, l2)
                x_out = np.average(x)
                y_out = np.average(y)
                z_out = np.average(z)
                self.img_out = cv2.drawContours(self.frontImage.copy(), ct, -1, (255, 0, 0), 2)
                pt = Point()
                pt.x = x_out
                pt.y = y_out
                pt.z = z_out
                self.pub_point.publish(pt)
                
                # I = Image()
                # I.height = 480
                # I.width = 640
                # I.header.stamp = rospy.Time.now()
                # I.header.frame_id = 'world'
                # I.encoding = 'uint8'
                # I.data = np.array(self.img_out, dtype = 'uint8').tolist()
                # print(I.data)
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(self.img_out))
                # print(ut_vector)
                # print(len(np.argwhere(self.count)))
                self.camera0.unregister()
                return True, pt, ut_vector
        return False, [0,0,0], [0,0,0]

class avoidance:
    def __init__(self):
        self.start_sim = False
        self.FOV = 80 # degrees
        self.max_depth = 10 # max depth of the camera
        self.goal_coordinate = 2.5 # only in x 
        self.distance_array = [] # stores the distance field of the centre points
        self.binary_array = [] # binary histogram
        self.distance_threshold = 1.35 # distance threshold for vision array (avoid if obstacle is closer than distance_threshold)
        self.step_size = 0.9
        self.reactiveness = 2
        self.max_reactiveness = 4
        self.k_c = 0.356
        self.x_to_move = 0
        self.y_to_move = 0
        self.k_th = self.k_c / (self.distance_threshold + (1/self.k_c))
        rospy.Subscriber('/red/challenge_started', Bool, self.start_challenge)
        time.sleep(0.1)
        self.camera0 = rospy.Subscriber('/red/camera/depth/image_raw', Image, self.camera_callback)
       
        time.sleep(0.1)


    def start_challenge(self, data):
        if data.data ==  True:
            # print(data.data)
            self.start_sim = True
    
    def camera_callback(self,data):
        # print(self.start_sim)
        # print('callback')
        if self.start_sim:
            self.image = ros_numpy.numpify(data)
            temp_array = []
            temp_dist_array = []
            for i in range(640):
                temp_distance = 0
                for j in range(238,241):
                    temp_distance = temp_distance + self.image[j][i]
                temp_array.append(temp_distance/3)
            k = 0
            for i in range(16):
                temp_average = 0
                for j in range(40):
                    temp_average = temp_average + temp_array[k]
                    k = k + 1 
                temp_dist_array.append(temp_average/40)                
            self.distance_array = np.copy(temp_dist_array)
            # print("Distance Array: ", self.distance_array)
            self.get_binary_array()
            self.get_refined_histograms()
            self.compute_theta()
            self.compute_heading_vectors()
            self.compute_reactiveness()
        

    def get_binary_array(self):
        temp_array = []
        for i in range(16):
            if math.isnan(self.distance_array[i]):
                temp_array.append(0)
            else :
                if self.distance_array[i] > self.distance_threshold:
                    temp_array.append(0)
                else:
                    temp_array.append(1)
        self.binary_array = np.copy(temp_array)
        # print ("Binary Array:",self.binary_array)

    def get_refined_histograms(self): # gets the most spacious sector from given binary array
        self.frequency_array = []
        self.refined_array = []
        self.refined_distance_array = []
        temp = self.binary_array[0]
        d_i = 0
        count = 0
        for i in range(len(self.binary_array)):
            
            if i == len(self.binary_array) - 1 and temp == self.binary_array[i]:
                self.frequency_array.append(count + 1)
                self.refined_array.append(self.binary_array[i])
                self.refined_distance_array.append((d_i+self.distance_array[i])/(count+1))

            elif i == len(self.binary_array) -1 and temp != self.binary_array[i]:
                self.frequency_array.append(count)
                self.refined_array.append(self.binary_array[i-1])
                self.refined_distance_array.append(d_i/count)
                self.frequency_array.append(1)
                self.refined_array.append(self.binary_array[i])
                self.refined_distance_array.append(self.distance_array[i])

            elif temp == self.binary_array[i]:
                count = count +1
                d_i = d_i + self.distance_array[i]
            
            else:
                self.frequency_array.append(count)
                self.refined_distance_array.append(d_i/count)
                count = 1
                self.refined_array.append(temp)
                temp = self.binary_array[i]
                d_i = self.distance_array[i]
        # print ("Frequency Array:", self.frequency_array)
        # print ("Refined distance array:", self.refined_distance_array)
    
    def compute_reactiveness(self):
        for i in range(len(self.refined_distance_array)):
            if math.isnan(self.refined_distance_array[i]):
                self.refined_distance_array[i] = 0
        # print (self.refined_distance_array)

        if (len(self.refined_distance_array) == 1):
            d_avg = 0
            
        elif (self.refined_starting_index == 0):
            d_avg = self.refined_distance_array[self.refined_starting_index + 1]

        elif (self.refined_starting_index == len(self.refined_distance_array)-1):
            d_avg = self.refined_distance_array[self.refined_starting_index - 1]
        
        else :
            d_avg = (self.frequency_array[self.refined_starting_index-1]*self.refined_distance_array[self.refined_starting_index-1] + self.frequency_array[self.refined_starting_index+1]*self.refined_distance_array[self.refined_starting_index+1])/(self.frequency_array[self.refined_starting_index-1]+self.frequency_array[self.refined_starting_index+1]) 

        reactiveness_temp = np.clip((1/(self.k_c + (self.k_th*(d_avg-self.distance_threshold)))), 0, self.max_reactiveness)
        self.reactiveness = reactiveness_temp
        # print ("Reactiveness :", reactiveness_temp)

    def compute_theta(self):
        index = 0
        if self.refined_array[0] == 0:
            separated_array = []
            for i in range(len(self.refined_array)):
                if i%2 == 0:
                    separated_array.append(self.frequency_array[i])
            max_freq = max(separated_array)
            index = 2*separated_array.index(max_freq) 
        elif self.refined_array[0] != 0:
            separated_array = []
            for i in range(len(self.refined_array)):
                if i%2 == 1:
                    separated_array.append(self.frequency_array[i])
            max_freq = max(separated_array)
            index = 2*separated_array.index(max_freq) + 1
        self.refined_starting_index = index
        sum = 0
        for i in range(index):
            sum = sum + self.frequency_array[i]
        optimal_index = sum + self.frequency_array[index]/2
        self.theta = ((180 - self.FOV)/2) + (optimal_index*self.FOV/16)
        # print ("Computed Theta:", self.theta)
             
    def compute_heading_vectors(self): # computing the heading vector with respect to the drones body frame
        self.x_hat = math.sin(math.radians(self.theta)) # front of the drone
        self.y_hat = math.cos(math.radians(self.theta)) # left of the drone
        self.x_to_move = self.step_size * self.x_hat # move this distance in x direction of the drone
        self.y_to_move = self.reactiveness * self.step_size * self.y_hat # move this distance in y direction of the dron

class FLIGHT_ICUAS:

    def __init__(self):

        #SUBSCRIBERS																	
        self.get_state_suscriber=rospy.Subscriber('/red/odometry', Odometry, self.get_state)
        self.ball_pose=rospy.Subscriber('/red/ball/odometry', Odometry, self.get_ball_state)

        #PUBLISHERS
        self.position_hold=rospy.Publisher('/red/position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=1)
        self.waypt_publisher= rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=1)
        self.balldrp=rospy.Publisher('/red/uav_magnet/gain', Float32, queue_size=10)

        self.x,self.y,self.z=0,0,0
        self.x_st,self.y_st,self.z_st=[],[],[]
        self.a,self.b,self.c,self.k=0,0,0,1
        self.vx,self.vy,self.vz=0,0,0
        self.wx,self.wy,self.wz=0,0,0

        self.bx,self.by,self.bz,self.bvx,self.bvy,self.bvz=0,0,0,0,0,0
        self.bxst,self.byst,self.bzst=[],[],[]
        self.bvxst,self.bvyst,self.bvzst=[],[],[]

        self.mnvr_state=False

    def get_state(self, odom_data):
        self.x,self.y,self.z=odom_data.pose.pose.position.x,odom_data.pose.pose.position.y,odom_data.pose.pose.position.z
        self.a,self.b,self.c,self.k=odom_data.pose.pose.orientation.x,odom_data.pose.pose.orientation.y,odom_data.pose.pose.orientation.z,odom_data.pose.pose.orientation.w
        self.vx,self.vy,self.vz=odom_data.twist.twist.linear.x,odom_data.twist.twist.linear.y,odom_data.twist.twist.linear.z
        self.wx,self.wy,self.wz=odom_data.twist.twist.angular.x,odom_data.twist.twist.angular.y,odom_data.twist.twist.angular.z

        # self.x_st.append(self.x)
        # self.y_st.append(self.y)
        # self.z_st.append(self.z)

    def spawn_ball(self):
        self.balldrp.publish(1)
        rospy.wait_for_service('/red/spawn_ball')
        #try:
        mode = rospy.ServiceProxy('/red/spawn_ball', Empty)
        response = mode()

    def get_ball_state(self,bst):
        self.bx,self.by,self.bz=bst.pose.pose.position.x,bst.pose.pose.position.y,bst.pose.pose.position.z
        self.bvx,self.bvy,self.bvz=bst.twist.twist.linear.x,bst.twist.twist.linear.y,bst.twist.twist.linear.z
        self.bxst.append(self.bx)
        self.byst.append(self.by)
        self.bzst.append(self.bz)
        self.bvxst.append(self.bvx)
        self.bvyst.append(self.bvy)
        self.bvzst.append(self.bvz)

    def go_waypt(self,gowpt,yaw):
        transforms =Transform()
        transforms.translation.x,transforms.translation.y,transforms.translation.z = gowpt       
        transforms.rotation.x,transforms.rotation.y,transforms.rotation.z,transforms.rotation.w = tf.transformations.quaternion_from_euler(0,0,yaw)          # 0,0,0,1 
        velocities =Twist()
        accelerations=Twist()
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(1)) #rospy.Duration(1)
        rate = rospy.Rate(20)
        for i in range(10):
            self.position_hold.publish(point)
            rate.sleep()

    def ball_launch(self,waypoint,normal,yaw):
        transforms =Transform()
        waypoint=waypoint-(3.5*normal)
        waypoint[2]=waypoint[2]+2
        transforms.translation.x,transforms.translation.y,transforms.translation.z= waypoint       #9,-3.0,4.0 
        transforms.rotation.x,transforms.rotation.y,transforms.rotation.z,transforms.rotation.w = tf.transformations.quaternion_from_euler(0,np.pi/2,yaw)    #0.0,0.5,0.0,0.866    #0,0.38268358785518847 ,0, 0.9238794681051637              #self.x,self.y,self.z
        velocities =Twist()
        velocities.linear.x,velocities.linear.y,velocities.linear.z=normal
        #velocities.angular.y=5
        accelerations=Twist()
        #accelerations.linear.x,accelerations.linear.y,accelerations.linear.z=-10,0,0
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Duration(1)) #,rospy.Duration(1)
        self.position_hold.publish(point)
        time.sleep(0.2)  #0.25
        self.balldrp.publish(0)
        # print(self.bx,self.by,self.bz,self.bvx,self.bvy,self.bvz)

        # waypoint[2]=waypoint[2]-1.1
        # self.go_waypt(waypoint,yaw)

    def ball_drop(self,locmarker,normal,yaw):

        waypoint=locmarker-(3.5*normal)
        waypoint[2]=waypoint[2]+0.5
        self.go_waypt(waypoint,yaw) 
        dist=  (pow(self.x-waypoint[0],2)+ pow(self.y-waypoint[1],2)+ pow(self.z-waypoint[2],2))                                       #(pow(self.x-11.7,2)+ pow(self.y-(-3),2)+ pow(self.z-2.9,2))
        while dist>0.2:
            dist=(pow(self.x-waypoint[0],2)+ pow(self.y-waypoint[1],2)+ pow(self.z-waypoint[2],2)) 
            #print('moving',dist)
            continue

        location=locmarker
        location[2]=location[2]+0.5
        #lx,ly,lz=location[0],location[1],location[2]+0.9
        #nx,ny,nz=normal[0],normal[1],normal[2]
        self.go_waypt(location,yaw)                 #(12.5,-3,2.9)
        #sleep(3)
        
        vecpt=location-(0.8*normal)
        dist=  (pow(self.x-vecpt[0],2)+ pow(self.y-vecpt[1],2)+ pow(self.z-vecpt[2],2))                                       #(pow(self.x-11.7,2)+ pow(self.y-(-3),2)+ pow(self.z-2.9,2))
        while dist>0.1:
            dist=(pow(self.x-vecpt[0],2)+ pow(self.y-vecpt[1],2)+ pow(self.z-vecpt[2],2)) 
            #print('moving',dist)
            continue
        self.ball_launch(locmarker,normal,yaw)


        
        #print(self.bxst[self.bxst.index(max(self.bxst))],self.bxst[self.byst.index(max(self.bxst))],self.bxst[self.bzst.index(max(self.bxst))])

        #--------------------------------------------------------------------------------
    
        #ax = plt.axes(projection = '3d')
        #ax.plot3D(self.x_st, self.y_st, self.z_st,'b',label='Tracked trajectory')
        #ax.plot3D(self.bxst, self.byst, self.bzst,'b',label='Ball trajectory')
        
        # plt.plot(self.bvxst,self.bxst,'r')
        # plt.show()
        # plt.plot(self.bxst,self.bzst,'b')
        # plt.xlabel('x')
        # plt.ylabel('z')
        # # ax.axes.set_xlim3d(left=0, right=15)
        # # ax.axes.set_ylim3d(bottom=-5, top=5)
        # # ax.axes.set_zlim3d(bottom=0, top=6)
        # plt.title('Ball Flight ')
        # plt.legend()

        # plt.show()
        # plt.plot(self.bxst,self.byst)
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.show()
        time.sleep(5)
        self.spawn_ball()

def contour_generator(frame):
    test_img1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    test_blur = cv2.GaussianBlur(test_img1, (5, 5), 0)
    edge = cv2.Canny(test_blur, 75, 200)
    edge1 = copy.copy(edge)
    contour_list = list()


###python2 m 3 output hai
    _, cnts, h = cv2.findContours(edge1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if h is not None:
        index = list()
        for hier in h[0]:
            if hier[3] != -1:
                index.append(hier[3])

        # loop over the contours
        for c in index:
            peri = cv2.arcLength(cnts[c], True)
            approx = cv2.approxPolyDP(cnts[c], 0.02 * peri, True)

            if len(approx) > 4:
                peri1 = cv2.arcLength(cnts[c - 1], True)
                corners = cv2.approxPolyDP(cnts[c - 1], 0.02 * peri1, True)
                contour_list.append(corners)

        new_contour_list = list()
        for contour in contour_list:
            if len(contour) == 4:
                new_contour_list.append(contour)
        final_contour_list = list()
        for element in new_contour_list:
            if cv2.contourArea(element) < 3500 and 500 < cv2.contourArea(element):
                final_contour_list.append(element)

        return final_contour_list
    else:
        return []

def cv_out(img):
    ct = contour_generator(img)
    if len(ct)> 0:
        return True, ct
    else:
        return False, ct

def disp(data):
    img_rgb_left = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    img_bgr_left = img_rgb_left[...,::-1]
    _, r = cv_out(img_bgr_left)
    if _:
        w = cv2.drawContours(img_rgb_left.copy(), r, -1, (255, 0, 0), 2)
    else:
        w = img_bgr_left.copy()

    cv2.imshow('YOLO output', w)
    cv2.waitKey(1)
