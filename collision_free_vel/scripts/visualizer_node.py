#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped,Point 
from visualization_msgs.msg import Marker

# This class is to visualize data from a drone. 

class DroneVisualizer:

    #callback ual pose
    def _dronePoseCallback(self,data):
        self._pose = data

    def _droneVelCallback(self,data):
        self._vel = data

    def publishMarkers(self):

        drone_marker = Marker()
        drone_marker.header.frame_id = "/map"
        drone_marker.header.stamp = rospy.Time.now()
        drone_marker.id = ord(self._id[-1])
        drone_marker.ns = "uavs"
        drone_marker.type = Marker.MESH_RESOURCE
        drone_marker.mesh_resource = "package://robots_description/models/mbzirc/meshes/multirotor.dae"
        drone_marker.color.a = 1
        drone_marker.action = Marker.ADD
        drone_marker.pose = self._pose.pose
        drone_marker.scale.x = 0.001
        drone_marker.scale.y = 0.001
        drone_marker.scale.z = 0.001
        drone_marker.mesh_use_embedded_materials = True
        drone_marker.color.r = 0.0
        drone_marker.color.g = 0.0
        drone_marker.color.b = 0.0

        # Drone identifier        
        id_marker = Marker()
        id_marker.header.frame_id = "/map"
        id_marker.header.stamp = rospy.Time.now()
        id_marker.id = ord(self._id[-1])
        id_marker.ns = "uavs_state"
        id_marker.type = Marker.TEXT_VIEW_FACING
        id_marker.text = self._id
        id_marker.pose.position.z = self._pose.pose.position.z+2.0
        id_marker.pose.position.y = self._pose.pose.position.y
        id_marker.pose.position.x = self._pose.pose.position.x
        id_marker.color.a=1
        id_marker.color.r = 1.0
        id_marker.color.g = 1.0
        id_marker.color.b = 1.0         
        id_marker.scale.x = 1
        id_marker.scale.y = 1
        id_marker.scale.z = 1
        id_marker.mesh_use_embedded_materials = True

        # Drone goal
        goal_marker = Marker()
        goal_marker.header.frame_id = "/map"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.id = ord(self._id[-1])
        goal_marker.ns = "uavs_state"
        goal_marker.type = Marker.SPHERE
        goal_marker.text = self._id
        goal_marker.pose.position.z = self._goal[2] 
        goal_marker.pose.position.y = self._goal[1] 
        goal_marker.pose.position.x = self._goal[0] 
        goal_marker.color.a=1
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.scale.x = 1
        goal_marker.scale.y = 1
        goal_marker.scale.z = 1
        goal_marker.mesh_use_embedded_materials = True

        # Drone cylinder
        uav_cylinder = Marker()
        uav_cylinder.header.frame_id = "/map"
        uav_cylinder.header.stamp = rospy.Time.now()
        uav_cylinder.id = ord(self._id[-1])
        uav_cylinder.ns = "uavs"
        uav_cylinder.type = Marker.CYLINDER
        uav_cylinder.color.a = 0.5   
        uav_cylinder.color.r = 1.0
        uav_cylinder.color.g = 0.647
        uav_cylinder.color.b = 0
        uav_cylinder.action = Marker.ADD
        uav_cylinder.pose.position = self._pose.pose.position
        uav_cylinder.scale.x = 2*self._radius   # Diameter
        uav_cylinder.scale.y = 2*self._radius
        uav_cylinder.scale.z = 5.0
        uav_cylinder.mesh_use_embedded_materials = True
             
        # Publish arrow for velocity
        arrow = Marker()
        arrow.header.frame_id = "/map"
        arrow.header.stamp = rospy.Time.now()
        arrow.id = ord(self._id[-1])
        arrow.ns = "uavs"
        arrow.type = Marker.ARROW
        arrow.lifetime = rospy.Duration(0.15)
        arrow.color.a = 1   
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0        
        arrow.action = Marker.ADD
        arrow.points = []
        arrow.points.append(Point(self._pose.pose.position.x,self._pose.pose.position.y,self._pose.pose.position.z))
        arrow.points.append(Point(self._pose.pose.position.x+self._vel.twist.linear.x*2,self._pose.pose.position.y+self._vel.twist.linear.y*2,self._pose.pose.position.z))       
        arrow.scale.x=0.1
        arrow.scale.y = 0.2
                
        self._vel_pub.publish(arrow)
        self._pose_pub.publish(drone_marker)             
        self._cylinder_pub.publish(uav_cylinder)   
        self._id_pub.publish(id_marker)
        self._goal_pub.publish(goal_marker)            


    #class initialization
    def __init__(self, id, goal, radius):
        
        self._id = id
        self._vel = TwistStamped()
        self._pose = PoseStamped()
        self._goal = goal
        self._radius = radius

        rospy.Subscriber(self._id + '/ual/pose',PoseStamped, self._dronePoseCallback,queue_size=1)
        rospy.Subscriber(self._id + '/ual/set_velocity',TwistStamped, self._droneVelCallback,queue_size=1)

        self._pose_pub = rospy.Publisher(self._id + '/pose_marker',Marker,queue_size=1)
        self._goal_pub = rospy.Publisher(self._id + '/goal_marker',Marker,queue_size=1)
        self._vel_pub = rospy.Publisher(self._id + '/vel_marker',Marker,queue_size=1)
        self._cylinder_pub = rospy.Publisher(self._id + '/cylinder_marker',Marker,queue_size=1)   
        self._id_pub = rospy.Publisher(self._id + '/id_marker',Marker,queue_size=1)
    
if __name__== "__main__":

    rospy.init_node('visualizer_node')

    conf_error = False
    if(rospy.has_param('~n_drones')):
        n_drones = rospy.get_param('~n_drones')
    else:
        conf_error = True
    
    if(rospy.has_param('~goals')):
        goals = rospy.get_param('~goals')
    else:
        conf_error = True

    if(conf_error):
        rospy.logerr("Missing basic configuration parameters.")
    elif(len(goals) < n_drones):
        rospy.logerr("Missing parameters for some drones.")
    else:   

        radius = rospy.get_param('~uav_radius', 3.0)
            
        drones = []

        for i in range(n_drones):
            drones.append(DroneVisualizer('uav_' + str(i+1),goals[i],radius))

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            for i in range(n_drones):
                drones[i].publishMarkers()

            rate.sleep()

