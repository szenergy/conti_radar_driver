#ifndef COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_
#define COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ars_408_can_defines.h"
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

#include <radar_conti/ObjectList.h>
#include <radar_conti/ClusterList.h>
#include <radar_conti/RadarState.h>
#include <radar_conti/ClusterStatus.h>
#include <radar_conti/Frame.h>


#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <math.h>
#include <cstddef>



typedef unsigned char ubyte;
typedef unsigned short int uword;

class Radar_Conti
{
public:
    Radar_Conti(const ros::NodeHandle &nh);
    ~Radar_Conti() = default;
    //create can_receive_callback
    void can_frame_callback(const can::Frame &msg);
    void init(can::DriverInterfaceSharedPtr &driver_);

private:


    ros::NodeHandle nh;
    //create CAN channel object
    can::DriverInterfaceSharedPtr driver_;
    can::FrameListenerConstSharedPtr frame_listener_;
    //can::DriverInterfaceSharedPtr radar_can_output;
    //create Publisher
    ros::Publisher pub_marker;
    ros::Publisher pub_objects;
    ros::Publisher pub_cluster;
    ros::Publisher pub_cluster_list;
    std::string pub_marker_array_topic_name = "/ars408/marker_array";
    // std::string pub_object_list_topic_name = "/ars408/objectlist";
    // std::string pub_tf_topic_name = "/tf";
    std::string frame_id_ = "/radar";
    
    void handle_cluster_list(const can::Frame &msg);

    //create handle_object_list
    void handle_object_list(const can::Frame &msg);
    //create publish_object_map
    void publish_object_map();

    void publish_cluster_map();
    //create map container for object list
    std::map<int,radar_conti::Object> object_map_;

    std::map<int,radar_conti::Cluster> cluster_map_;
    

    //create data structures for radar object list
    radar_conti::ObjectList object_list_;
    radar_conti::ClusterList cluster_list;

    //additional variables
    int operation_mode_;

    //Object Classes in strings
    const std::string object_classes[8] = {"Point","Car","Truck","Pedestrian","Motorcycle","Bicycle","Wide","Reserved"};
};

#endif  // COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_