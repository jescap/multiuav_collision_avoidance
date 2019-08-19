#!/usr/bin/env python
import rospy
from uav_abstraction_layer.srv import TakeOff, Land
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 
import time
from numpy import linalg as LA
import math

class DroneNavigator:

    #callback ual pose
    def dronePoseCallback(self,data):
        self.pose_ = data

    def getDronePose(self):
        return self.pose_ 

    #class initialization
    def __init__(self, id, height):
        
        self.id_ = id
        self.height_ = height

        self.vel_pub_ = rospy.Publisher(id_ + '/ual/set_velocity',TwistStamped,queue_size=1)
        #self.pose_pub_ = rospy.Publisher(id_+'/ual/set_pose',PoseStamped,queue_size=1)
        
        rospy.Subscriber(id_ + '/ual/pose',PoseStamped, self.dronePoseCallback,queue_size=1)
    
        self.takeoff_srv_name_ = id_ + '/ual/take_off'
        self.land_srv_name_ = id_ + '/ual/land'

        
    def collision_avoidance_function(self):
        ####################################################
        ### INCLUDE COLLISION AVOIDANCE FUNCTION HERE#############
        ####################################################
        #Drone 1 pose -> self.pose_list[1].pose.position.x
                   # self.pose_list[1].pose.position.y
                   # self.pose_list[1].pose.position.z
        #Drone 2 pose -> self.pose_list[2].pose.position.x
                    #self.pose_list[2].pose.position.y
                    #self.pose_list[2].pose.position.z
                # ........
        collision = False
        if(collision):
            self.vel.twist.linear.x = 0.0
            self.vel.twist.linear.y = 0.0
            self.vel.twist.linear.z = 0.0
            return True
        else:
            return False
   
    
    def cal_distance_to_waypoint(self,x,y,z):
        distance_to_waypoint = 1
        aux = LA.norm([self.pose_list[self.drone].pose.position.x-x, self.pose_list[self.drone].pose.position.y-y,self.pose_list[self.drone].pose.position.z-z])        
        if aux>distance_to_waypoint:
            return False
        else:
            return True

    def takeOff(self):
        try:
            take_off_client = rospy.ServiceProxy(self.takeoff_srv_name_, TakeOff)
            take_off_client.call(self.height_,True)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"%e)

    def land(self):
        try:
            land_client = rospy.ServiceProxy(self.land_srv_name_, Land)
            land_client.call(True)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"%e)

    def mission_thread(self):
        threading.Thread(target=self.mission).start()
    
    def mission(self):
        for i in self.waypoints:
            while(self.cal_distance_to_waypoint(i[0],i[1],i[2])==False):
                if(self.collision_avoidance_function()):
                    print("Collision detected")
                    self.vel_pub(self.vel)
                else:
                    pos = PoseStamped()
                    pos.header.frame_id = 'map'
                    pos.pose.position.x = i[0]
                    pos.pose.position.y = i[1]
                    pos.pose.position.z = i[2]
                    pos.pose.orientation.w = 1
                    self.pose_pub.publish(pos)
                time.sleep(0.1)
        try:
            land_client = rospy.ServiceProxy(self.land_service, Land)
            land_client.call(True)
            print("UAV ",self.drone,": landing")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                    

if __name__== "__main__":

    rospy.init_node('centralized_avoidance')

    # Reading parameters    
    conf_error = False

    if(rospy.has_param('~n_drones')):
        n_drones = rospy.get_param('~n_drones')
    else:
        conf_error = True

    if(rospy.has_param('~heights')):
        heights = rospy.get_param('~heights')
    else:
        conf_error = True
    
    if(rospy.has_param('~goals')):
        heights = rospy.get_param('~goals')
    else:
        conf_error = True
    
    if(conf_error):
        rospy.logerr("Missing configuration parameters.")
    elif(heights.len() < n_drones || goals.len() < n_drones):
        rospy.logerr("Missing parameters for some drones.")
    else:

        # Create and takeoff drones
        drones = []
        for i in range(n_drones):
            drones.append(DroneNavigator('uav_'+str(i+1)), heights[i])

        for i in range(n_drones):
            drones[i].takeOff()

        print "\nPress a key to start the mission:\n"
        key = raw_input(" >> ")

        # TODO
        for i in range(n_drones):
            drones[i].mission_thread()
        


