#!/usr/bin/env python
import rospy
from uav_abstraction_layer.srv import TakeOff, Land
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 

from auxiliar_functions import *
from uav import UAV
from algorithms import bf_minimize_max_deviation


# This class is an interface with the actual drones. 

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
        self._vel_pub.publish(self._vel)    

    #class initialization
    def __init__(self, id, height):
        
        self._id = id
        self._height = height
        self._pose = PoseStamped()

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


# This class implements the central controller resolving all conflicts and commading drones.

class CentralController:

    def __init__(self):

        # Reading conf parameters    
        conf_error = False

        if(rospy.has_param('~n_drones')):
            self._n_drones = rospy.get_param('~n_drones')
        else:
            conf_error = True

        if(rospy.has_param('~heights')):
            self._heights = rospy.get_param('~heights')
        else:
            conf_error = True
        
        if(rospy.has_param('~goals')):
            self._goals = rospy.get_param('~goals')

            ## For experimental test. Permutation of goals
            perm = [1, 0, 3, 2]
            goals_aux = self._goals
            self._goals = []
            for i in range(self._n_drones):
                self._goals.append(goals_aux[perm[i]])
        else:
            conf_error = True
        
        if(conf_error):
            rospy.logerr("Missing basic configuration parameters.")
        elif(len(self._heights) < self._n_drones or len(self._goals) < self._n_drones):
            rospy.logerr("Missing parameters for some drones.")
        else:

            # Create drone navigators
            self._navigators = []
            for i in range(self._n_drones):
                self._navigators.append(DroneNavigator('uav_' + str(i+1), self._heights[i]))


            # Reading algorithm parameters
            self._uav_radius = rospy.get_param('~uav_radius', 3.0)            # Radius of the UAVs
            self._k = rospy.get_param('~n_directions', 10)                    # Number of directions per UAV
            self._time_horizon = rospy.get_param('~time_horizon', 10)         # Time horizon (seconds)
            self._coll_det_time = rospy.get_param('~coll_detection_time', 5)  # Time to check conflicts again (seconds)
            self._max_deviation = rospy.get_param('~max_deviation', 0.785)    # Maximum deviation allowed (radians)
            self._rate = rospy.get_param('~rate',10)                          # Node rate (Hz)
            self._speed = rospy.get_param('~nominal_speed', 1.0)               # Nominal speed for UAVs (m/s)

            # Create UAVs models
            self._uavs = []
            for id in range(self._n_drones):
                self._uavs.append(UAV((0,0,0),self._speed,self._uav_radius,(1,1,1),self._goals[id]))
            
            self._uavs_idx = range(self._n_drones)

    # Main thread
    def execute(self):

        detect_method = lambda UAV1, d1, UAV2, d2 : is_collision_on_interval(UAV1, d1, UAV2, d2, self._time_horizon)
        cost_function = lambda x, y: abs(vector2angles(x)[0] -  vector2angles(y)[0])
        vect_dist = vectors_distance_by_components

        # Takeoff drones
        rospy.loginfo("Taking off drones.")

        for i in range(self._n_drones):
                self._navigators[i].takeOff()

        print "\nPress a key to start the mission:\n"
        key = raw_input(" >> ")
    
        rate = rospy.Rate(self._rate) 

        n_counts = 0
        avg_time = 0.0
        n_iterations = 0
        max_iterations = self._rate*self._coll_det_time

        while self._uavs and not rospy.is_shutdown():
            
            # Update UAVs positions
            arrived = []    
            for id,uav in enumerate(self._uavs):

                pos = self._navigators[self._uavs_idx[id]].getDronePose()
                uav.position = (pos.pose.position.x,pos.pose.position.y,pos.pose.position.z)
                uav.direction = uav.get_optimal_direction()

                # Check drones at goal
                if(vect_dist(uav.position, uav.goal_point) < 1.5):
                    arrived.append(id)

            # Removed drones at goal from collision avoidance
            for i in range(len(arrived)):
                self._uavs.pop(arrived[-(i+1)])
                idx = self._uavs_idx.pop(arrived[-(i+1)])
                self._navigators[idx].setDroneVel([0,0,0])
                rospy.loginfo("Drone %s at destination, stopping.", str(idx))

            # Check conflicts and solve them
            if n_iterations == 0 and detect_collisions_on_time_interval(self._uavs, self._time_horizon):

                print "Checking conflicts\n"

                time = rospy.Time.now()

                directions = [uav.generate_directions2D(self._max_deviation, self._k) for uav in self._uavs]
                
                result,no = bf_minimize_max_deviation(self._uavs, directions, cost_function, detect_method)
               
                time = rospy.Time.now() - time
                n_counts += 1
                avg_time += time.to_sec()

                if not result:
                    rospy.logwarn("No solution for conflict found. Stopping.")

                    break

                print "Solution found.\n"
                for uav, d in result:
                    uav.direction = d
                    print "Direction: " + str(d)

            # Send velocities
            for id,uav in enumerate(self._uavs):

                vel = [uav.velocity*uav.direction[0],uav.velocity*uav.direction[1],uav.velocity*uav.direction[2]]
                self._navigators[self._uavs_idx[id]].setDroneVel(vel)

            n_iterations += 1
            if n_iterations == max_iterations:
                n_iterations = 0

            rate.sleep()


        # Stop UAVs just in case
        for nav in self._navigators:
            nav.setDroneVel([0,0,0])

        if(n_counts > 0):
            avg_time = avg_time/n_counts
        
        print "Average computation time: " + str(avg_time) + " for " + str(n_counts) + " calls\n"

        print "\nPress a key to end the mission:\n"
        key = raw_input(" >> ")
    
        rospy.loginfo("Landing drones.")
        
        for nav in self._navigators:
            nav.land()


if __name__== "__main__":

    rospy.init_node('central_avoidance_node')

    contrl = CentralController()
    contrl.execute()        


