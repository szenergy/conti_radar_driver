#include <iostream>
#include <ros/ros.h>
#include "ars_408_can_defines.h"
#include "visibility_control.h"
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <socketcan_bridge/socketcan_to_topic.h>
#include <can_msgs/Frame.h>



#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <math.h>
#include <cstddef>

#define _USE_MATH_DEFINES

ros::Publisher pub;

using namespace std;
typedef unsigned char ubyte;
typedef unsigned short int uword;


void can_receive_callback(const can::Frame msg)
{
    int operation_mode_= 0;
    if (msg.id == ID_RadarState) {
        operation_mode_ =CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(msg.data),1.0);

        ROS_INFO("Radar_Outputype: %d", operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_NVMReadStatus(GET_RadarState_RadarState_NVMReadStatus(msg.data),1.0);

        ROS_INFO("Radar_NVM_read: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_MaxDistanceCfg(GET_RadarState_RadarState_MaxDistanceCfg(msg.data),1.0);

        ROS_INFO("Radar_Max_Distance: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_SortIndex(GET_RadarState_RadarState_SortIndex(msg.data),1.0);

        ROS_INFO("Radar_Sort_Index: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_RadarPowerCfg(GET_RadarState_RadarState_RadarPowerCfg(msg.data),1.0);

        ROS_INFO("Radar_Power: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_SendQualityCfg(GET_RadarState_RadarState_SendQualityCfg(msg.data),1.0);

        ROS_INFO("Radar_send_Quality_info: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_SendExtInfoCfg(GET_RadarState_RadarState_SendExtInfoCfg(msg.data),1.0);

        ROS_INFO("Extended information for Object: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_SendQualityCfg(GET_RadarState_RadarState_SendQualityCfg(msg.data),1.0);

        ROS_INFO("Radar send quality information: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_MotionRxState(GET_RadarState_RadarState_MotionRxState(msg.data),1.0);

        ROS_INFO("Radar_inputs: %d",operation_mode_);

        operation_mode_ = CALC_RadarState_RadarState_RCS_Threshold(GET_RadarState_RadarState_RCS_Threshold(msg.data),1.0);

        ROS_INFO("Radar_RCS_thresshold: %d", operation_mode_);

    }
    

}


int main(int argc, char **argv)
{

        ros::init(argc, argv, "lidar_filt");
        ros::NodeHandle nh;

        can::DriverInterfaceSharedPtr radar_output = std::make_shared<can::ThreadedSocketCANInterface> ();

        if(!radar_output->init("can0",0)) 
        {
                ROS_FATAL("Failed to initialize can_device at CAN0");
                return 1;
        }

        can::FrameListenerConstSharedPtr frame_listener_ = radar_output->createMsgListener(can_receive_callback);

        ros::spin();
}