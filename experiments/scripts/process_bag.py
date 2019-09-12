#!/usr/bin/env python
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np

import rosbag

n_drones = 4
uav_poses = {}
min_dist = -1.0
travelled_distance = []
uav_found = {}
uav_traj = []

for i in range(n_drones):
    uav_poses['/uav_'+str(i+1)+'/ual/pose'] = ()
    uav_found['/uav_'+str(i+1)+'/ual/pose'] = 0
    travelled_distance.append(0.0)
    uav_traj.append([])

print "\nIntroduce the name of the bag file:\n"
name = raw_input(" >> ")
    
bag = rosbag.Bag(name)

for topic, msg, t in bag.read_messages():

    if topic.find('/ual/pose') != -1:

        uav_poses[topic] = (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z) 
        uav_found[topic] = 1

        if(sum(uav_found.values()) == n_drones):

            # All poses available, record time step
            for i in range(n_drones):
                uav_traj[i].append(uav_poses['/uav_'+str(i+1)+'/ual/pose'])

            # Empty uavs
            for i in range(n_drones):
                uav_poses['/uav_'+str(i+1)+'/ual/pose'] = ()
                uav_found['/uav_'+str(i+1)+'/ual/pose'] = 0    
        
# Process trajectories

for id in range(n_drones):
    for j in range(1,len(uav_traj[id])):

        x1,y1,z1 = uav_traj[id][j]
        x0,y0,z0 = uav_traj[id][j-1]  
        dist = sqrt((x0-x1)**2+(y0-y1)**2)
        travelled_distance[id] += dist

d_min = -1
for step in range(len(uav_traj[0])):
        
    for id1 in range(n_drones-1):
        for id2 in range(id1+1,n_drones):
            x1,y1,z1 = uav_traj[id1][step]
            x0,y0,z0 = uav_traj[id2][step]  
            dist = sqrt((x0-x1)**2+(y0-y1)**2)
            if d_min == -1 or dist < d_min:
                d_min = dist 

# Print results and plots
print "The minimum distance between drones is: " + str(d_min) + "\n"

for id in range(n_drones):
    print "Distance travelled by drone " + str(id+1) + ": " + str(travelled_distance[id]) + "\n" 

UAV1 = np.array(uav_traj[0][:])
UAV2 = np.array(uav_traj[1][:])
UAV3 = np.array(uav_traj[2][:])
UAV4 = np.array(uav_traj[3][:])

plt.plot(UAV1[:,0], UAV1[:,1], 'r-', label='UAV 1')
plt.plot(UAV2[:,0], UAV2[:,1], 'b-', label='UAV 2')
plt.plot(UAV3[:,0], UAV3[:,1], 'g-', label='UAV 3')
plt.plot(UAV4[:,0], UAV4[:,1], 'm-', label='UAV 4')
plt.plot(UAV1[-1,0], UAV1[-1,1], 'r*')
plt.plot(UAV2[-1,0], UAV2[-1,1], 'b*')
plt.plot(UAV3[-1,0], UAV3[-1,1], 'g*')
plt.plot(UAV4[-1,0], UAV4[-1,1], 'm*')

plt.xlabel('X (m)')
plt.ylabel('Y (m)')

plt.title("UAV Trajectories")
#plt.legend()
plt.show()

bag.close()