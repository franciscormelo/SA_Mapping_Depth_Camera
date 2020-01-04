#!/usr/bin/env python

###############################################################################
#
#                           Autonomous Systems - 2019/2020 IST
#
#                       Mapping with Depth Camera ROS Node
#   Authors:
#       - Carolina Costa 84175
#       - Francisco Melo, 84053
#       - Raul Vaqueiro, 84089
#       - Rodrigo Rego, 89213
#
#
#
###############################################################################

from as_mapping.my_ros_independent_class import my_generic_sum_function

import math
import numpy as np

# ROS python api with lots of handy ROS functions
import rospy

# sensor msg
from sensor_msgs.msg import LaserScan

# pose msg
from geometry_msgs.msg import PoseWithCovarianceStamped

# for Occupancy grid msgs
from nav_msgs.msg import OccupancyGrid

# for msgs time info
from std_msgs.msg import Header


# to Synchronize
import message_filters

# to measure blocks of code running time
import time

# for transformations
import tf

class occupancy_grid_mapping():
    def __init__(self):
        '''
        Class constructor.
        '''
        rospy.init_node('mapping', anonymous=False)
        # print message in terminal
        rospy.loginfo('Occupancy Grid Mapping Started')

        # Subscribe Laser msg topico /scan msg Object - LaserScan
        rospy.Subscriber("/scan_depth", LaserScan,self.callbackLaser,queue_size=1)


        # Subscribe pose msg topico /amcl_pose msg Object - PoseWithCovarianceStamped
        rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.callbackPose,queue_size=1)

        # pose and laser flags
        self.pose_msg_received = False
        self.laser_msg_received = False

        # Set ROS Node Rate
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        # Laser Parameters initialization
        self.max_range = None
        self.min_range = None
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.ranges = None

        # Pose Parameters initialization
        self.cov = None
        self.quartenion = None
        self.pos_x = 0
        self.pos_y = 0

        # MAP publisher
        self.map_pub = rospy.Publisher('/map_mapping',OccupancyGrid, queue_size=0)

        # Map Parametes
        self.width = 1500
        self.height = 1500
        self.resolution = 0.02

        # Map Initialization
        #log odds map
        self.lt = np.zeros((self.width, self.height))
        #probability map
        self.pb =  (np.ones((self.width, self.height))*(-1))

        # Create occupancy grid msg object to publish
        self.map = OccupancyGrid()


        # Map message information
        self.map.info.resolution=self.resolution
        self.map.info.width=self.width
        self.map.info.height=self.height
        self.map.info.origin.position.x=-5
        self.map.info.origin.position.y=-5
        self.map.info.origin.position.z=0
        self.map.info.origin.orientation.x=0
        self.map.info.origin.orientation.y=0
        self.map.info.origin.orientation.z=0
        self.map.info.origin.orientation.w=0

        # Map data must be a list -> Matrix to list conversion
        for i in range (0,self.width):
            for j in range(0,self.height):
                self.map.data.append(self.pb[i][j])

        #Publish initial map
        self.map_pub.publish(self.map)


        # Mapping constants
        self.l0 = 0 # Default log probability
        self.locc = np.log(0.73/(1-0.73))
        self.lfree = np.log(0.27/(1-0.27))

        self.alpha = 0.1 # Thickness of obstacles
        self.beta = None # rad # Angular width of the sensor beam
        self.zmax = None # meters # Max reading from the sensor
        self.zmin = None

        self.sensor_theta = None

        # Map offset
        self.offset = 20.0

    def callbackPose(self, msg_pose):
        '''
        This function gets executed everytime a laser scan msg
        receives a msg_laser object.
        '''

        # Time of pose msg
        self.pose_t = msg_pose.header.stamp.secs

        #sets flag of pose messaged received = true
        self.pose_msg_received = True
        # Pose Parameters
        self.cov = msg_pose.pose.covariance
		# Quartenion orientation
        self.quartenion = [msg_pose.pose.pose.orientation.x, msg_pose.pose.pose.orientation.y, msg_pose.pose.pose.orientation.z, msg_pose.pose.pose.orientation.w]
		# 2D Position
        self.pos_x = msg_pose.pose.pose.position.x
        self.pos_y = msg_pose.pose.pose.position.y


    def callbackLaser(self, msg_laser):
        '''
        This function gets executed everytime a amcl pose msg receives
        a msg_pose object.
        '''
        self.laser_t = msg_laser.header.stamp.secs
        self.laser_msg_received = True
        # Laser Parameters
        self.max_range = msg_laser.range_max
        self.min_range = msg_laser.range_min
        self.angle_min = msg_laser.angle_min
        self.angle_max = msg_laser.angle_max
        self.angle_increment = msg_laser.angle_increment
        self.ranges = msg_laser.ranges

        self.beta = self.angle_max - self.angle_min # rad # Angular width of the sensor beam
        self.zmax = self.max_range
        self.zmin = self.min_range

        #angular spacing between beams
        self.sensor_theta=np.linspace(self.angle_min, self.angle_max,len(self.ranges))



    def mapping(self):
        '''
        Mappin method.
        '''
        #counts the amount of time spent in mapping algorithm
############################################
        start = time.time()
###########################################

     # Robot 2D Pose (x,y, yaw)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.quartenion)

        x = (self.pos_x + self.offset)*(1/self.resolution)
        y = (self.pos_y + self.offset)*(1/self.resolution)
        xt = [x, y, yaw]
        zt = self.ranges # Set of measurements


        self.occupancy_grid_mapping(xt, zt)

        self.pb = 100*(1 - (1 / (1 + np.exp(self.lt)))) # log odds -> probability

        #celss outisde cone have 50% probability of being occupied
        outside_pos = self.pb == 50

        # cell outside the laser cone are marked unknown, in this case, -1
        self.pb[outside_pos] = -1


        occupied_thr = (self.pb > 50) # occupied cell treshold
        free_thr = (self.pb <= 50) & (self.pb >=0) # free cell treshold


        self.pb[occupied_thr] = 100 # assings 100% probability to map occupied cells
        self.pb[free_thr] = 0 # assings 0% probability to map occupied cells


        self.map.data = [] # map data list intialization


        # converts map matrix to a list to publish purposes
        map_data = np.reshape(self.pb,(1, self.width*self.height))
        self.map.data = map_data.tolist()[0]

##########################################
        end = time.time()
        time_taken = end - start
        print('Time: ',time_taken)
##########################################
        # publishes map_mapping topic to ROS
        self.map_pub.publish(self.map)




    def occupancy_grid_mapping(self, xt, zt): # z - mesurement | xt = <x,y,theta>
        '''
            Occupancy grid algorithm method.
        '''

        for i in range(len(zt)): # for all measurements
            zi = zt[i]
            if not np.isnan(zi) and zi <= self.zmax and zi>=self.zmin:
                theta_k = self.sensor_theta[i]
                x0 = xt[0] # x pos of the robot
                y0 = xt[1] # y pos of the robot


                theta = xt[2] + theta_k; #orientation of the robot + beam angle


                if theta > np.pi:
                    theta -= 2 * np.pi
                elif theta < -np.pi:
                    theta += 2 * np.pi

                # Position of the obstacle
                x1 = x0 + ((zi) * math.cos(theta)*(1/self.resolution))
                y1 = y0 + ((zi) *math.sin(theta)*(1/self.resolution))

                # Position of the obstacle + object thickness alpha
                x2 = x0 + ((zi + self.alpha)*math.cos(theta)*(1/self.resolution))
                y2 = y0 +((zi + self.alpha)*math.sin(theta)*(1/self.resolution))


                x0 = int(round(xt[0]))
                y0 = int(round(xt[1]))
                x1 = int(round(x1))
                y1 = int(round(y1))
                x2 = int(round(x2))
                y2 = int(round(y2))

                # Bresenham’s Line Algorithm
                # Cells in the line from x0,y0 to x1,x2 are marked free
                self.plotLine(x0, y0, x1, y1, self.lfree)
                # Cells in the line from x1,y1 to x2,y2 are marked occupied
                self.plotLine(x1, y1, x2, y2, self.locc)


    def plotLine(self, x0, y0, x1, y1, lupdate):
        '''
            Plots a line in a Matrix. Is the slope is positive it calls the
            plotLineLow otherwise it calls the polotLineHigh.
        '''


        if abs(y1 - y0) < abs(x1 - x0):
            if x0 > x1:
                self.plotLineLow(x1, y1, x0, y0, lupdate)
            else:
                self.plotLineLow(x0, y0, x1, y1, lupdate)
        else:
            if y0 > y1:
                self.plotLineHigh(x1, y1, x0, y0, lupdate)
            else:
                self.plotLineHigh(x0, y0, x1, y1, lupdate)

    def plotLineLow(self, x0, y0, x1, y1, lupdate):
        '''
            For gradients between 0 and 1 by checking whether y needs to
            increase or decrease. Draws a line in the matrix.
        '''
        dx = x1 - x0
        dy = y1 - y0
        xi = 1
        yi = 1

        if dx < 0:
            xi = -1

        if dy < 0:
            yi = -1
            dy = -dy

        D = 2*dy - dx
        y = y0

        # Updates the cells in the line from the log odds matrix
        for x in range(x0, x1+1, xi):
            self.lt[x][y] +=lupdate- self.l0

            if D > 0:
               y = y + yi
               D = D - 2*dx

            D = D + 2*dy

    def plotLineHigh(self, x0, y0, x1, y1, lupdate):
        '''
        For gradients between 0 and 1 by checking whether x needs to increase
        or decrease . Draws a line in a matrix.
        '''
        dx = x1 - x0
        dy = y1 - y0
        xi = 1
        yi = 1

        if dx < 0:
            xi = -1
            dx = -dx

        if dy < 0:
            yi = 1

        D = 2*dx - dy
        x = x0
            # Updates the cells in the line from the log odds matrix
        for y in range(y0, y1+1, yi):
            self.lt[x][y] += lupdate - self.l0

            if D > 0:
               x = x + xi
               D = D - 2*dy

            D = D + 2*dx
    def compare_timestamps(self):
        '''
        Ensure that no big time difference exists between msgs for sync purposes.
        '''


        diff = abs(self.pose_t- self.laser_t)
        print(diff)
        if diff >=1: # ensure difference betwen msg times is less than 1
            return False
        else:
            return True


    def run_behavior(self):
        '''
            Algorithm Master.
        '''
        while not rospy.is_shutdown():
            if self.pose_msg_received == True: #checks for pose msgs
                rospy.loginfo('pose msg received!')
                if self.laser_msg_received == True: #checks for laser msgs
                    rospy.loginfo('laser msg received!')
                    # lower flags
                    self.laser_msg_received = False
                    self.pose_msg_received = False
                    # compare timestamps
                    if self.compare_timestamps(): # Check if msgs are in sync
                        self.mapping() # Runs the mapping algorithm
                    else:
                        rospy.logwarn('pose and laser msgs are not in sync')




def main():
    mapp = occupancy_grid_mapping()
    mapp.run_behavior()
    print 'success'

if __name__ == '__main__':
    main()
