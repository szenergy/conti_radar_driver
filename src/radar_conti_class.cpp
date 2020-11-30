#include "radar_conti_ars408_component.hpp"

Radar_Conti::Radar_Conti(const ros::NodeHandle &nh) : nh(nh) {};

void Radar_Conti::init(can::DriverInterfaceSharedPtr &driver_)
{
    pub = nh.advertise<visualization_msgs::MarkerArray>("radar_objects",0);
    this->driver_ = driver_;
    this->frame_listener_ = driver_->createMsgListenerM(this,&Radar_Conti::can_frame_callback);
}

void Radar_Conti::can_frame_callback(const can::Frame &msg)
{
    int operation_mode_= 0;
    handle_object_list(msg);
    //std::cout << msg.id << " " << ID_RadarState << "\n";
    if (msg.id == ID_RadarState) {
        operation_mode_ =CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(msg.data),1.0);

    }
    if (operation_mode_ == 0) {

    }

    
}
void Radar_Conti::handle_object_list(const can::Frame &msg)
{
    //std::cout << msg.id << " " << ID_Obj_0_Status << "\n";
    if (msg.id == ID_Obj_0_Status) {

        publish_object_map();

        object_list_.header.stamp = ros::Time::now();
        object_list_.object_count.data = GET_Obj_0_Status_Obj_NofObjects(msg.data);

        object_map_.clear();


    }


    if (msg.id == ID_Obj_1_General) {

        radar_conti::Object o;

        //object ID
        int id = GET_Obj_1_General_Obj_ID(msg.data);
        o.obj_id.data = GET_Obj_1_General_Obj_ID(msg.data);

        // //RCLCPP_INFO(this->get_logger(), "Object_ID: 0x%04x", o.obj_id.data);

        //longitudinal distance
        o.object_general.obj_distlong.data =
                CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(msg.data), 1.0);

        //lateral distance
        o.object_general.obj_distlat.data =
                CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(msg.data), 1.0);

        //relative longitudinal velocity
        o.object_general.obj_vrellong.data =
                CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(msg.data), 1.0);

        //relative lateral velocity
        o.object_general.obj_vrellat.data =
                CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(msg.data), 1.0);

        o.object_general.obj_dynprop.data =
                CALC_Obj_1_General_Obj_DynProp(GET_Obj_1_General_Obj_DynProp(msg.data), 1.0);

        o.object_general.obj_rcs.data = 
                CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(msg.data), 1.0);

        //insert object into map
        if(o.object_general.obj_rcs.data > 1.0)
        {
               object_map_.insert(std::pair<int, radar_conti::Object>(id, o));
        }
    }

    if (msg.id == ID_Obj_2_Quality) {

        int id = GET_Obj_2_Quality_Obj_ID(msg.data);


        object_map_[id].object_quality.obj_distlong_rms.data =
                CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_distlat_rms.data =
                CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_vrellong_rms.data =
                CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_vrellat_rms.data =
                CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(msg.data), 1.0);

    }
    if (msg.id == ID_Obj_3_Extended) {

        int id = GET_Obj_3_Extended_Obj_ID(msg.data);


        object_map_[id].object_extended.obj_arellong.data =
                CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(msg.data), 1.0);

        object_map_[id].object_extended.obj_arellat.data =
                CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(msg.data), 1.0);

        object_map_[id].object_extended.obj_class.data =
                CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

        int obj_class = CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

        object_map_[id].object_extended.obj_orientationangle.data =
                CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(msg.data),
                                                            1.0);

        object_map_[id].object_extended.obj_length.data =
                CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(msg.data), 1.0);

        object_map_[id].object_extended.obj_width.data =
                CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(msg.data), 1.0);
    }

}
void Radar_Conti::publish_object_map() {
        
        std::map<int, radar_conti::Object>::iterator itr;

        // tf::Transform transforms;

        // for (itr = object_map_.begin(); itr != object_map_.end(); ++itr) {

        //         std::string tf_name = "object_" + std::to_string(itr->first);

        //         geometry_msgs::TransformStamped tf;

        //         tf.header.stamp = ros::Time::now();
        //         tf.header.frame_id = frame_id_;

        //         tf.child_frame_id = tf_name;


        //         transforms.setOrigin(tf::Vector3(itr->second.object_general.obj_distlong.data,itr->second.object_general.obj_distlat.data,1.0));

        //         tf.transform.translation.x = itr->second.object_general.obj_distlong.data;
        //         tf.transform.translation.y = itr->second.object_general.obj_distlat.data;
        //         tf.transform.translation.z = 1.0;

        //         transforms.setRotation(tf::Quaternion(0.0,0.0,0.0));

        //         tf.transform.rotation.w = 1.0;
        //         tf.transform.rotation.x = 0.0;
        //         tf.transform.rotation.y = 0.0;
        //         tf.transform.rotation.y = 0.0;

        //         transforms.transforms.push_back(tf);


        //         object_list_.objects.push_back(itr->second); 

        // }


        // object_list_publisher_->publish(object_list_);
        // tf_publisher_->publish(transforms);

        visualization_msgs::MarkerArray marker_array;
        // marker_array.markers.clear();

        // //delete old marker
        // visualization_msgs::Marker ma;
        // ma.action=3;
        // marker_array.markers.push_back(ma);
        // pub.publish(marker_array);
        // marker_array.markers.clear();

        //marker for ego car
        visualization_msgs::Marker mEgoCar;

        mEgoCar.header.stamp = ros::Time::now();
        mEgoCar.header.frame_id = frame_id_;
        mEgoCar.ns = "";
        mEgoCar.id = 999;

        //if you want to use a cube comment out the next 2 line
        mEgoCar.type = 1; // cube
        mEgoCar.action = 0; // add/modify
        mEgoCar.pose.position.x = -2.0;
        mEgoCar.pose.position.y = 0.0;
        mEgoCar.pose.position.z = 1.0;

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, M_PI/2);

        mEgoCar.pose.orientation.w = myQuaternion.getW();
        mEgoCar.pose.orientation.x = myQuaternion.getX();
        mEgoCar.pose.orientation.y = myQuaternion.getY();
        mEgoCar.pose.orientation.z = myQuaternion.getZ();
        mEgoCar.scale.x = 1.0;
        mEgoCar.scale.y = 1.0;
        mEgoCar.scale.z = 1.0;
        mEgoCar.color.r = 0.0;
        mEgoCar.color.g = 0.0;
        mEgoCar.color.b = 1.0;
        mEgoCar.color.a = 1.0;
        mEgoCar.lifetime = ros::Duration(0.4);
        mEgoCar.frame_locked = false;

        marker_array.markers.push_back(mEgoCar);

        for (itr = object_map_.begin(); itr != object_map_.end(); ++itr) {

                if(itr->second.object_general.obj_rcs.data > 1)
                {
                        visualization_msgs::Marker mobject;
                        visualization_msgs::Marker mtext;

                        mtext.header.stamp = ros::Time::now();
                        mtext.header.frame_id = frame_id_;
                        mtext.ns = "";
                        mtext.id = (itr->first+100);
                        mtext.type = 1; //Cube
                        mtext.action = 0; // add/modify
                        mtext.pose.position.x = itr->second.object_general.obj_distlong.data;
                        mtext.pose.position.y = itr->second.object_general.obj_distlat.data;
                        mtext.pose.position.z = 4.0;

                
                        //myQuaternion.setRPY(M_PI / 2, 0, 0);
                        myQuaternion.setRPY(0, 0, 0);

                        mtext.pose.orientation.w = myQuaternion.getW();
                        mtext.pose.orientation.x = myQuaternion.getX();
                        mtext.pose.orientation.y = myQuaternion.getY();
                        mtext.pose.orientation.z = myQuaternion.getZ();
                        // mtext.scale.x = 1.0;
                        // mtext.scale.y = 1.0;
                        mtext.scale.z = 2.0;
                        mtext.color.r = 1.0;
                        mtext.color.g = 1.0;
                        mtext.color.b = 1.0;
                        mtext.color.a = 1.0;
                        mtext.lifetime = ros::Duration(0.4);
                        mtext.frame_locked = false;
                        mtext.type=9;
                        mtext.text= "object_" + std::to_string(itr->first) + ": \n" 
                        + " RCS: " + std::to_string(itr->second.object_general.obj_rcs.data) + "dBm^2" + " \n" 
                        + " V_long: " +   std::to_string(itr->second.object_general.obj_vrellong.data) + "m/s" + " \n" 
                        + " V_lat: " + std::to_string(itr->second.object_general.obj_vrellat.data) + "m/s" + " \n" 
                        + " Orientation: " + std::to_string(itr->second.object_extended.obj_orientationangle.data) + "degree" + " \n"
                        + " Class: " + object_classes[itr->second.object_extended.obj_class.data] + "\n";


                        marker_array.markers.push_back(mtext);



                        mobject.header.stamp = ros::Time::now();
                        mobject.header.frame_id = frame_id_;
                        mobject.ns = "";
                        mobject.id = itr->first;
                        mobject.type = 1; //Cube
                        mobject.action = 0; // add/modify
                        mobject.pose.position.x = itr->second.object_general.obj_distlong.data;
                        mobject.pose.position.y = itr->second.object_general.obj_distlat.data;
                        mobject.pose.position.z = 1.0;

                        myQuaternion.setRPY(0, 0, 0);

                        mobject.pose.orientation.w = myQuaternion.getW();
                        mobject.pose.orientation.x = myQuaternion.getX();
                        mobject.pose.orientation.y = myQuaternion.getY();
                        mobject.pose.orientation.z = myQuaternion.getZ();
                        mobject.scale.x = itr->second.object_extended.obj_length.data;
                        mobject.scale.y = itr->second.object_extended.obj_width.data;
                        mobject.scale.z = 1.0;
                        mobject.color.r = 0.0;
                        mobject.color.g = 1.0;
                        mobject.color.b = 0.0;
                        mobject.color.a = 1.0;
                        mobject.lifetime = ros::Duration(0.4);
                        mobject.frame_locked = false;

                        marker_array.markers.push_back(mobject);
                }
        

        }
        pub.publish(marker_array);

}

