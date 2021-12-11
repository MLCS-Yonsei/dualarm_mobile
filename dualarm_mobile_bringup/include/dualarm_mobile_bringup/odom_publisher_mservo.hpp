#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ethercat_test/vel.h>
#include <dualarm_sensor_msgs/as5047Msg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <math.h>

#define _USE_MATH_DEFINES

// Sensor message types
using ethercat_test::vel;
using dualarm_sensor_msgs::as5047Msg;
using nav_msgs::Odometry;
using geometry_msgs::Twist;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;


class OdomPublisher{
    
    public:
    void initiate_variables();
    void publisher_set();
    void callback_camera(const Odometry::ConstPtr& camera_msg);
    void callback_enc(const vel::ConstPtr& enc_msg);
    void subscriber_set();
    void calculate_velocity_position();
    void velocity_pose_publish();
    MatrixXd getRotMat(double angle);

    double yaw = 0;

    Vector4d measure_val;
    MatrixXd Jacob = MatrixXd(3,4);

    Vector3d v_robot;    

    Vector3d v_est;
    Vector3d p_est_curr;
    Vector3d p_est_prev;

    Quaterniond q_cam = Quaterniond::Identity();
    Quaterniond q_est = Quaterniond::Identity();
    Quaterniond q_init = Quaterniond::Identity();

    private:
    
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher publisher_odom;
    ros::Publisher publisher_twist;
    ros::Subscriber subscriber_camera;
    ros::Subscriber subscriber_encoder;

    // ROS Time info
    double t_curr = 0;
    double dt = 1/200.0;
    double t_prev = ros::Time::now().toSec();
    // Mobile robot spec
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double l_x = 0.21658;
    const double l_y = 0.1645;
    const double l_sep = l_x + l_y;

    // Motor spec and unit conversion ratio
    const double gear_ratio = 74.5;
    const double rpm_to_radps = 2.0*M_PI/60.0;

};

void OdomPublisher::initiate_variables()
{
    measure_val.setZero();
    
    Jacob<<r/4.0, r/4.0, r/4.0, r/4.0,
        -r/4.0, r/4.0, -r/4.0, r/4.0,
        -r/4.0/l_sep, r/4.0/l_sep, r/4.0/l_sep, -r/4.0/l_sep;
    
    v_robot.setZero();
    
    v_est.setZero();
    p_est_curr.setZero();
    p_est_prev.setZero();
}

void OdomPublisher::publisher_set()
{
    publisher_odom = nh_.advertise<Odometry>("odom",1);
}

void OdomPublisher::callback_camera(const Odometry::ConstPtr& camera_msg)
{
    q_cam.w() = camera_msg->pose.pose.orientation.w;
    q_cam.x() = camera_msg->pose.pose.orientation.x;
    q_cam.y() = camera_msg->pose.pose.orientation.y;
    q_cam.z() = camera_msg->pose.pose.orientation.z;

    double qw = q_cam.w();
    double qx = q_cam.x();
    double qy = q_cam.y();
    double qz = q_cam.z();
    
    //yaw = atan2( 2*( q_cam.w()*q_cam.z()+q_cam.x()*q_cam.y() )
    //,1-2*( q_cam.y()*q_cam.y()+q_cam.z()*q_cam.z() ) );
   yaw = atan2(2*(qw*qz + qx*qy),1-2*(qy*qy+qz*qz));
    calculate_velocity_position();
    velocity_pose_publish();
}

void OdomPublisher::callback_enc(const vel::ConstPtr& enc_msg)
{
    measure_val << enc_msg->velocity[0], -(enc_msg->velocity[1]),
                    -(enc_msg->velocity[2]), enc_msg->velocity[3];
}

void OdomPublisher::subscriber_set()
{
    subscriber_camera = nh_.subscribe("/camera/odom/sample",1,&OdomPublisher::callback_camera,this);
    subscriber_encoder = nh_.subscribe("/measure",1,&OdomPublisher::callback_enc,this);
}

void OdomPublisher::calculate_velocity_position()
{
    v_robot = Jacob * measure_val/gear_ratio*rpm_to_radps;
    v_est = getRotMat(yaw)*v_robot;

    p_est_curr = p_est_prev + v_est*dt;
    p_est_curr(2) = yaw;
    p_est_prev = p_est_curr;
}

void OdomPublisher::velocity_pose_publish()
{
    t_curr = ros::Time::now().toSec();
    dt = t_curr - t_prev;

    Odometry odom;
    Twist vel;

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = p_est_curr(0);
    odom.pose.pose.position.y = p_est_curr(1);
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation.w = cos(yaw/2);
    odom.pose.pose.orientation.x = 0;    
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = sin(yaw/2);

    odom.twist.twist.linear.x = v_robot(0);
    odom.twist.twist.linear.y = v_robot(1);
    odom.twist.twist.angular.z = v_robot(2);
    
    vel.linear.x = v_robot(0);
    vel.linear.y = v_robot(1);
    vel.angular.z = v_robot(2);

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.0001;
    odom.pose.covariance[28] = 0.0001;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001) {
      odom.pose.covariance[35] = 0.0001;
    }else{
      odom.pose.covariance[35] = 0.01;
    }

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[14] = 0.01;
    odom.twist.covariance[21] = 0.0001;
    odom.twist.covariance[28] = 0.0001;
		
    if (abs(odom.twist.twist.angular.z) < 0.0001) {
      odom.twist.covariance[35] = 0.0001;
    }else{
      odom.twist.covariance[35] = 0.01;
    }


    publisher_odom.publish(odom);    
    publisher_twist.publish(vel);
}

MatrixXd OdomPublisher::getRotMat(double angle)
{
    MatrixXd R = MatrixXd(3,3);
    R << cos(angle), -sin(angle), 0,
        sin(angle), sin(angle), 0,
        0,  0,  1;

    return R;
}