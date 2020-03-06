import numpy as np
import rospy, rosbag, tf, sys

from geometry_msgs.msg import Quaternion

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# script to make coordinate tfs

def initializeAruco(N=12):
    
    arucos = np.zeros((N,7)) # array with [x y z x y z w] (pose w/ quaternions)
    
    for i in range(arucos.shape[0]):
        # position
        arucos[i,0] = i*2 + 1.0         #x
        if i % 2 == 1:                # if odd
            arucos[i,1] = 2.16 - 0.126  #y if odd
        else:
            arucos[i,1] = 0.126         #y if even
        
        arucos[i,2] = 0.115             #z
        
        # orientation in quaternions
        arucos[i,3] = -0.5  # x
        arucos[i,4] = -0.5  # y 
        arucos[i,5] = 0.5   # z
        arucos[i,6] = 0.5   # w
    
    return arucos

def quaternionToHomogeneous(quat):
    
    #   Converts a Nx7 matrix containing (x, y, z, x, y, z, w) to 
    # a 4x4xN homogeneous transformation matrix

    N = quat.shape[0]
    H = np.zeros((4,4,N))
    print(H.shape)
    
    for i in range(N):
        
        # translation
        t = np.array([quat[i,0], quat[i,1], quat[i,2]]).transpose()

        # rotation
        x = quat[i,3]
        y = quat[i,4]
        z = quat[i,5]
        w = quat[i,6]

        R =[[1-2*(y**2+z**2), 2*(x*y-w*z)    , 2*(x*z+w*y)],
            [2*(x*y+w*z)    , 1-2*(x**2+z**2), 2*(y*z-w*x)],            
            [2*(x*z-w*y)    , 2*(y*z+w*x)    , 1-2*(x**2+y**2)]]

        H[0:3,0:3, i] = R
        H[0:3, 3 , i] = t
        H[ 3 ,0:4, i] = np.array([0.0, 0.0, 0.0, 1.0])

    return H

if __name__ == '__main__':


    paths = ["bags/200212_154801_corredor_lab_reta.bag","bags/200212_160813_corredor_lab_ziguezague.bag","bags/200212_165047_corredor_lab_reta_180.bag",
    "bags/200212_171052_corredor_lab_reta_ziguezague.bag"]

    if (len(sys.argv) <= 1):
        print("Usage: python <filename.py> bagfile_number      (from 0 to 3)")

    # you should pass which bagfile you want on the command line, right after the script name
    bagfile_number = int(sys.argv[1])

    bag = rosbag.Bag(paths[bagfile_number])

    # we need 3 tfs
    # 1 - world  -> ArUco      - measured
    # 2 - ArUco  -> camera     - given by ar_track_alvar (but camera -> QR)
    # 3 - camera -> robot      - static tf


    ## tf #1 - world  -> ArUco
    N = 12                                                      # number of arucos
    quat_arucos = initializeAruco(N)                            # Nx7 - [x, y, z, x, y, z, w]
    tf_world_arucos = quaternionToHomogeneous(quat_arucos)      # 4x4xN
    print("tf_world_arucos" + str(tf_world_arucos.shape))

    ## tf #2 - ArUco -> camera (or actually, camera -> ArUco)

    total_markers = 0       # iteration counter
    id_seq = [] # stores the identified aruco's IDs
    try:

        for topic, msg, t in bag.read_messages(topics=['/ar_pose_marker']):

            if ( np.array(msg.markers).shape[0] > 0 and msg.markers[0].id != 0): # checking if we detected any marker   
                
                # get camera frame_id
                camera_frame_id = msg.markers[0].header.frame_id
                #print(camera_frame_id) #axis_front_optical_frame
                # always getting the first marker, if more than one is detected
                x = msg.markers[0].pose.pose.position.x
                y = msg.markers[0].pose.pose.position.y
                z = msg.markers[0].pose.pose.position.z

                xx = msg.markers[0].pose.pose.orientation.x
                yy = msg.markers[0].pose.pose.orientation.y
                zz = msg.markers[0].pose.pose.orientation.z
                ww = msg.markers[0].pose.pose.orientation.w

                aux = np.array([[x, y, z, xx, yy, zz, ww]])

                try: 
                    tf_camera_arucos = np.append(tf_camera_arucos, aux, axis=0)
                except:
                    print("fail")
                    tf_camera_arucos = aux
                    

                # saving IDs for each identified tag
                # didnt initialize it previously with zeros(size(tf_aux,1),1)
                # because we dont know exacty how many we're using, since we have
                # empty messages (that didnt recognize any markers)
                id_seq.append(msg.markers[0].id)
                total_markers = total_markers + 1       # counts every time we identify a marker

                #print(strcat("ID = ",num2str(msg.markers(1).id), " and ~= ",num2str(msg.markers(1).id ~= 0)))
            

    except:
        print("Error")

    # transform to Homogeneous
    tf_camera_arucos = quaternionToHomogeneous(tf_camera_arucos) # 4x4xN
    print("tf_camera_arucos" + str(tf_camera_arucos.shape))
    id_seq = np.array([id_seq])


    ## tf #3 - camera -> robot

    tf_camera_base_link = np.zeros((1,7))
    tf_camera_base_link[0, 0] = 0
    tf_camera_base_link[0, 1] = 0.1750
    tf_camera_base_link[0, 2] = -0.2700

    tf_camera_base_link[0, 3] = 0.5
    tf_camera_base_link[0, 4] = -0.5
    tf_camera_base_link[0, 5] = 0.5
    tf_camera_base_link[0, 6] = 0.5


    tf_camera_base_link = quaternionToHomogeneous(tf_camera_base_link) # 4x4xN
    tf_camera_base_link.resize(tf_camera_base_link.shape[0],tf_camera_base_link.shape[1])
    print("tf_camera_base_link" + str(tf_camera_base_link.shape))

    ## transforms
    tf_world_base_link = np.zeros((4,4,total_markers))

    count = 0
    for i in range(total_markers): # iterate only on messages that we saw a marker
        
        aruco_id = id_seq[0,i]
        if(aruco_id < 12):
            
            #print("tf_world_arucos[:,:,aruco_id].shape" + str(tf_world_arucos[:,:,aruco_id].shape))
            #print("np.linalg.inv(tf_camera_arucos[:,:,i]).shape" + str(np.linalg.inv(tf_camera_arucos[:,:,i]).shape))
            #print("tf_camera_base_link.shape" + str(tf_camera_base_link.shape))
            

            tf_world_base_link[:,:,i] = tf_world_arucos[:,:,aruco_id] * np.linalg.inv(tf_camera_arucos[:,:,i]) * tf_camera_base_link

            # if (sum(tf_world_base_link[:,:,count]) == 0):
            #     print(aruco_id)
            #     tf_world_arucos[:,:,aruco_id]
            #     np.linalg.inv(tf_camera_arucos[:,:,count])
            #     tf_camera_base_link

    ## plot

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #ax = fig.gca(projection='3d')

    x = tf_world_base_link[0, 3, :]
    y = tf_world_base_link[1, 3, :]
    z = tf_world_base_link[2, 3, :]

    ax.scatter(x, y, z, label='parametric curve')
    ax.legend()

    plt.show()


    # for i in range(tf_world_base_link.shape[2]):
    #     x = tf_world_base_link[0, 3, i]
    #     y = tf_world_base_link[1, 3, i]
    #     z = tf_world_base_link[2, 3, i]

    #     ax.plot(x, y, z, label='parametric curve')
    #     ax.legend()

    #     plt.show()