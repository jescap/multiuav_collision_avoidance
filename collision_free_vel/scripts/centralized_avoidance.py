#!/usr/bin/env python
import rospy
from uav_abstraction_layer.srv import TakeOff, Land
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 

import time

class DroneNavigator:

    #callback ual pose
    def _dronePoseCallback(self,data):
        self._pose = data

    def getDronePose(self):
        return self._pose 

    def setDroneVel(self, vel):
        self._vel.twist.linear.x = vel[0]
        self._vel.twist.linear.y = vel[1]
        self._vel.twist.linear.z = vel[2]
        self._vel_pub(self._vel)    

    #class initialization
    def __init__(self, id, height):
        
        self._id = id
        self._height = height

        self._vel = TwistStamped()
        self._vel_pub = rospy.Publisher(self._id + '/ual/set_velocity',TwistStamped,queue_size=1)
        #self._pose_pub = rospy.Publisher(self._id+'/ual/set_pose',PoseStamped,queue_size=1)
        
        rospy.Subscriber(self._id + '/ual/pose',PoseStamped, self._dronePoseCallback,queue_size=1)
    
        self._takeoff_srv_name = self._id + '/ual/take_off'
        self._land_srv_name = self._id + '/ual/land'

        
    def takeOff(self):
        try:
            take_off_client = rospy.ServiceProxy(self._takeoff_srv_name, TakeOff)
            take_off_client.call(self._height,True)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"%e)

    def land(self):
        try:
            land_client = rospy.ServiceProxy(self._land_srv_name, Land)
            land_client.call(True)
        except rospy.ServiceException, e:
            rospy.logwarn("Service call failed: %s"%e)



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
    elif(len(heights) < n_drones || len(goals) < n_drones):
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
        


