#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Empty
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(10)
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    
    LEAF_SIZE = 0.004
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    
    cloud_filtered = vox.filter()
    
    # TODO: PassThrough Filter
    # z axis
    passthrough = cloud_filtered.make_passthrough_filter()
    
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.607
    axis_max = 1.2
    passthrough.set_filter_limits (axis_min, axis_max)
    
    cloud_filtered = passthrough.filter()
    
    # y axis
    passthrough = cloud_filtered.make_passthrough_filter()
    
    filter_axis = 'y'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = -0.457
    axis_max = 0.457
    passthrough.set_filter_limits (axis_min, axis_max)
    
    cloud_filtered = passthrough.filter()
    
    passthrough_pub.publish(pcl_to_ros(cloud_filtered))
    
    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    
    max_distance = 0.005
    seg.set_distance_threshold(max_distance)
    
    inliers, coefficients = seg.segment()
    
    # TODO: Extract inliers and outliers
    pcl_cloud_objects = cloud_filtered.extract(inliers, negative=True)
    pcl_cloud_table = cloud_filtered.extract(inliers, negative=False)
    
    unclustered_objects_pub.publish(pcl_to_ros(pcl_cloud_objects))
    table_pub.publish(pcl_to_ros(pcl_cloud_table))
    
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(pcl_cloud_objects)    
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    #Tolerance is basically the distance criteria between points)
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(10000)
    
    # Use the k-d tree as search method
    tree = white_cloud.make_kdtree()
    ec.set_SearchMethod(tree)
    
    # Extract indices for each of the discovered clusters. Return type is a list of lists of indices
    cluster_indices = ec.Extract()
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    
    # TODO: Publish ROS messages
    pcl_cluster_pub.publish(ros_cluster_cloud)
    
# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = pcl_cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
        
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
        
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    """
    collision_map_pub.publish(ros_cloud_table)
    rospy.sleep(5)
    collision_map_prox = rospy.ServiceProxy('/clear_octomap', Empty())
    collision_map_prox()
    rospy.sleep(5)
    """
    
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
    
# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):

    # TODO: Initialize variables
    labels = []
    centroids = []
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    dict_list = []
    
    # TODO: Get/Read parameters
    picked_object_list = rospy.get_param('/object_list')
        
    # TODO: Parse parameters into individual variables
    for object in detected_objects:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        mean_pos = np.mean(points_arr, axis=0)[:3]
        mean_pos2 = []
        
        for coordinate in mean_pos:
            mean_pos2.append(np.asscalar(coordinate))
            
        centroids.append(mean_pos2)
    
    test_scene_num.data = 1
    
    
    # TODO: Rotate PR2 in place to capture side tables for the collision map
    
    # TODO: Loop through the pick list
    for picked_object in picked_object_list:
    
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        object_name.data = picked_object['name']
        
        try:
            index = labels.index(object_name.data)
            pick_pose.position.x = centroids[index][0]
            pick_pose.position.y = centroids[index][1]
            pick_pose.position.z = centroids[index][2]
        except ValueError:
            print(object_name.data + ' is not detected')
            continue
        
        # TODO: Create 'place_pose' for the object
        dropbox_pos = rospy.get_param('/dropbox')
        
        if picked_object['group'] == 'green':
            place_pose.position.x = dropbox_pos[1]['position'][0]
            place_pose.position.y = dropbox_pos[1]['position'][1]
            place_pose.position.z = dropbox_pos[1]['position'][2]
        else:
            place_pose.position.x = dropbox_pos[0]['position'][0]
            place_pose.position.y = dropbox_pos[0]['position'][1]
            place_pose.position.z = dropbox_pos[0]['position'][2]
        
        # TODO: Assign the arm to be used for pick_place
        if picked_object['group'] == 'green':
            arm_name.data = 'right'
        else:
            arm_name.data = 'left'
        
        # TODO: Publish pointcloud for collision map
        
        
        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)
        
        """
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        """
        
    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_' + str(test_scene_num.data) + '.yaml', dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    
    # TODO: Create Publishers
    passthrough_pub = rospy.Publisher("/passthrough", PointCloud2, queue_size=1)
    table_pub = rospy.Publisher("/table", PointCloud2, queue_size=1)
    unclustered_objects_pub = rospy.Publisher("/unclustered_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    
    collision_map_pub = rospy.Publisher("/pr2/3d_map/points", PointCloud2, queue_size=1)
    
    world_joint_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64, queue_size=1)
    
    # TODO: Load Model From disk
    model = pickle.load(open('project_model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

