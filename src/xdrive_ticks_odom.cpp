
#include <vector>
#include <string>
#include <iostream>

#include <stdio.h>
// send cmd 'via'.so, get Ticks4wheel 'via .so', publish /odom topic '
// and publish 'the odom/ base_link TransformStamped'msg to tf.
//
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

#include "robbase_msg/encoders.h"
#include "robbase_msg/WheelSpeed.h"
// #include "xdrive_ticks_cmd/WheelSpeed.h"
// #include "encoder_test/ticks.h"

// #include "xdriver.h"
#include "xdrive_ticks_cmd/xdriver.h"

ros::Time current_time, last_time;

#define M_PIpi 3.1415926
double base_width, ticks_per_meter;
// #define wheel_base 0.496
float reduction_ratio;
double d_diam;
bool first_tick_try_flag;
bool valid_first_tick_flag; ///
double left_ticks_prev, right_ticks_prev;

int right_wheel_speed;
int left_wheel_speed;

float var_Vlin, var_Vang;

ros::NodeHandle *private_n;
// xdrive_ticks_cmd::WheelSpeed wheelspeed_msg;
robbase_msg::WheelSpeed wheelspeed_msg;
// typedef struct { float Vlin; float Vang; } twist_struct;
// twist_struct twist_vec_struct;

double self_x=0;
double self_y=0;
double self_th=0;
double left_ticks, right_ticks;
double delta_left_ticks, delta_right_ticks;
double elapsed_dt;

typedef struct{
	int tick_rf;
	int tick_lb;
	int tick_lf;
	int tick_rb;
} tick_vec_struct;
tick_vec_struct tick_vec;

float lwheelmotor, rwheelmotor;
ros::Publisher cmd_vel_pub;

using namespace std;
/*
void wheel_update_Speed_callback(const xdrive_unit::WheelSpeed& vel_motors_msg){
    d_diam = 0.14;
    left_wheel_speed = vel_motors_msg.lwheelmotor/d_diam*M_PIpi*reduction_ratio*60;
    right_wheel_speed = vel_motors_msg.rwheelmotor/d_diam*M_PIpi*reduction_ratio*60;
} */
// #if 1

void cmdvel_Callback(const geometry_msgs::Twist::ConstPtr& cmdvel_msg)
{
  // vel_x_struct.Vlin = cmdvel_msg->linear.x;
  // vel_w_struct.Vang = cmdvel_msg->angular.z;
  var_Vlin = cmdvel_msg->linear.x;
  var_Vang = cmdvel_msg->angular.z;

  // Vlinr = (lmotor + rmotor) / 2
  // Vangular = (rmotor - lmotor) / WMR_width
  lwheelmotor = var_Vlin - var_Vang * base_width *0.5;
  rwheelmotor = var_Vlin + var_Vang * base_width *0.5;
  // .header.frame_id = "/qch";
  wheelspeed_msg.lwheel = (-1) * lwheelmotor /d_diam*M_PIpi*reduction_ratio*60 ;
  wheelspeed_msg.rwheel = rwheelmotor /d_diam*M_PIpi*reduction_ratio*60;
  wheelspeed_msg.header.stamp = ros::Time::now();
  // header
  cmd_vel_pub.publish(wheelspeed_msg);
}
// #if 1 # endif // a=JunZhen

int main(int argc, char** argv)
{
    char strBuf[100];
    first_tick_try_flag = true;
    valid_first_tick_flag = false;

	// struct twist_struct vel_9_struct;
	var_Vlin =0.0;
	var_Vang =0.0;

    ros::init(argc, argv, "xdrive_ticks_odom_node" );
    ROS_INFO("xdrive_ticks_odom_node starting");

    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~").
    private_n= new ros::NodeHandle("~");

    // double base_width;
    if(!private_n->getParam("base_width", base_width)) {
        ROS_WARN("No base_width provided - default: 0.496m");
        base_width = 0.496;
    }

    int ticks_per_meter;
    // nh.param("ticks_per_meter", ticks_per_meter,157940);
    if(!private_n->getParam("ticks_per_meter", ticks_per_meter))  {
        ROS_WARN("No ticks_per_meter provided - default: 157940");
        ticks_per_meter = 157940;
    }

    double d_exec_rate;
    // nh_private.param("exec_rate", d_exec_rate, 20);
    if(!private_n->getParam("exec_rate", d_exec_rate)) {
        ROS_WARN("Not provided: exec_rate. Default=20");
        d_exec_rate = 20;
    }

    // float reduction_ratio;
    // nh_private.param("f_reduction_ratio", reduction_ratio, 65);
    if(!private_n->getParam("f_reduction_ratio", reduction_ratio)) {
        ROS_WARN("Not provided: f_reduction_ratio. Default=65");
        d_diam  = 65;
    }

    float d_diam;
    // nh_private.param("wheel_diam", d_diam, 0.262);
    if(!private_n->getParam("wheel_diam", d_diam)) {
        ROS_WARN("Not provided: d_diam. Default=0.262");
        d_diam  = 0.262;
    }

    bool rwheeltick_positive_;
    int rwheeltick_positive_factor =1;
    // flag: {rwheeltick_positive, lwheeltick_negative}, vice versa.
    if(!private_n->getParam("rwheeltick_positive_flag", rwheeltick_positive_)) {
        ROS_WARN("Not provided: rwheeltick_positive_flag. Default=true");
        rwheeltick_positive_ = true;
    }
    // nh_private.param("rwheeltick_positive_flag", rwheeltick_positive_, true);
    if (rwheeltick_positive_ == true)
      rwheeltick_positive_factor = 1;
    else if (rwheeltick_positive_ == false)
      rwheeltick_positive_factor = -1;

  ROS_INFO("serial connection _ not-use");
  ros::Subscriber cmdvel_sub = nh.subscribe("cmd_vel", 20, cmdvel_Callback);

  //  ros::Publisher ticksLR_pub = nh.advertise<robbase_msg::ticks>("/ticks", 20);
  ros::Publisher ticksLR_pub = nh.advertise<robbase_msg::encoders>("/ticksLR", 20);
  // ros::Publisher ticksLR4_pub = nh.advertise<encoder_test::ticks>("/ticks", 20);
  //  ros::Publisher ticksMLR_pub = nh.advertise<robbase_msg::RazorImu>("/ticksMLR", 20);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 20);

  // showing motor_speed:
  cmd_vel_pub  = nh.advertise<robbase_msg::WheelSpeed>("/wheelspeed", 10);
  //  cmd_vel_pub  = nh.advertise<robbase_msg::WheelSpeed>("/wheelspeed", 10);

  // struct qch.rwheelmotor

  // ros::Duration dur_time;
  robbase_msg::encoders ticksMsg;

  tf::TransformBroadcaster odom_tf_broadcaster;
  geometry_msgs::TransformStamped odom_transform_msg;
  //  encoder_test::ticks ticksMsg;
  ros::Rate loop_rate(d_exec_rate);

    last_time = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();
        double dx, dr, dist, dtheta, d_left, d_right;
        double x, y;
        // xdriver_struct_set("Vlin",&vel_x_struct);
        // xdriver_struct_set("Vang",&vel_w_struct;
        xdriver_setValue_float("Vlin", var_Vlin);
        xdriver_setValue_float("Vang", var_Vang);

        tick_vec.tick_lb = xdriver_getValue("ticklb") * rwheeltick_positive_factor * (-1);
        tick_vec.tick_lf  = xdriver_getValue("ticklf") * rwheeltick_positive_factor * (-1);
        tick_vec.tick_rb =  xdriver_getValue("tickrb") * rwheeltick_positive_factor;
        tick_vec.tick_rf = xdriver_getValue("tickrf") * rwheeltick_positive_factor;

        if (first_tick_try_flag == true) {
          left_ticks_prev = tick_vec.tick_lb;
          right_ticks_prev = tick_vec.tick_rb;
          first_tick_try_flag = false;
          // valid_first_tick_flag = false;
	    }
        // if (first_tick_try_flag == false && valid_first_tick_flag == false)
        //  valid_first_tick_flag = true;

        ticksMsg.header.stamp = ros::Time::now();
        ticksMsg.ticks_l = tick_vec.tick_lb;
        ticksMsg.ticks_r = tick_vec.tick_rb;
        // ticksMsg.ticks_r = int( (tick_vec.tick_rb + tick_vec.tick_rf) *0.5 );

        ticksLR_pub.publish(ticksMsg);
        // xdrive_motor();
        left_ticks = tick_vec.tick_lb;
        right_ticks = tick_vec.tick_rb;

        current_time = ros::Time::now();
        delta_left_ticks = left_ticks - left_ticks_prev;
        delta_right_ticks = right_ticks - right_ticks_prev;

        elapsed_dt = (current_time - last_time).toSec();

        // === compute odometry
        d_left = delta_left_ticks/ticks_per_meter;
        d_right= delta_right_ticks/ticks_per_meter;
        // distance traveled as average of both wheels
        // this approximation works (in radians) for small angles
        dist = (d_left + d_right)/2;
        dtheta = (d_right - d_left)/ base_width;
        //calculate velocities
        dx = dist / elapsed_dt;
        dr = dtheta / elapsed_dt;
        //calculate distance traveled and position
        if (dist != 0) {
            x = cos(dtheta) * dist;
            y = -sin(dtheta) * dist;
            //calculate position
            self_x += (cos(self_th) * x - sin(self_th) * y);
            self_y += (sin(self_th) * x + cos(self_th) * y);
        }
        if (dtheta != 0) {
            self_th += dtheta;
        }
        // We use a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(self_th);
        // odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,self_th);

        // publish TransformStamped message for odom/base_link 'to topic /tf
        odom_transform_msg.header.frame_id = "odom";
        odom_transform_msg.child_frame_id = "base_link";
        odom_transform_msg.header.stamp = current_time;
        odom_transform_msg.transform.translation.x = self_x;
        odom_transform_msg.transform.translation.y = self_y;
        odom_transform_msg.transform.translation.z = 0.0;
        odom_transform_msg.transform.rotation = odom_quat;

        // publish the /odom topic
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //set the position
        odom.pose.pose.position.x = self_x;
        odom.pose.pose.position.y = self_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = dx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = dr;
        // pose.cov
        odom.pose.covariance[0]  = 0.01;
        odom.pose.covariance[7]  = 0.01;
        odom.pose.covariance[14] = 99999;
        odom.pose.covariance[21] = 99999;
        odom.pose.covariance[28] = 99999;
        odom.pose.covariance[35] = 0.01;
        odom.twist.covariance = odom.pose.covariance;

        odom_pub.publish(odom);

        // publishing the odometry and the tf: odom/ base_link
        odom_tf_broadcaster.sendTransform(odom_transform_msg);

        last_time = current_time;
        right_ticks_prev = right_ticks;
        left_ticks_prev = left_ticks;

        // if (first_tick_try_flag == false && valid_first_tick_flag == false)
        //  valid_first_tick_flag = true;

        // ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop _ ( ros::ok() )
}// end.main()
