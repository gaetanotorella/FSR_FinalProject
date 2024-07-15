#include "geometric_controller/utils_fun.h"

Vector3d e_3(0, 0, 1);
float g = 9.81;


geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
Matrix3d Rb;
float u_T;
Matrix<double, 3, 3> Kp;
Matrix<double, 3, 3> Kv;
Vector3d zb_des;
Vector3d err_p;
Vector3d err_v;
Vector3d ang_vel;


void odomCallback(const nav_msgs::Odometry& msg);
void imuCallback(const sensor_msgs::Imu& msg);

void outer_loop (UAVParameters &uav_params, Vector3d pos_des, Vector3d vel_des, Vector3d acc_des){

    Vector3d err_pos = PoseToEigen(pose) - pos_des;
    Vector3d dot_err_pos = TwistLinearToEigen(twist) - vel_des;

    err_p = err_pos;
    err_v = dot_err_pos;
    
    ROS_INFO_STREAM("OUTER err_pos: " << err_pos.transpose());

    u_T = -(-Kp * err_pos -Kv * dot_err_pos - uav_params.mass * g * e_3 + uav_params.mass * acc_des).transpose() * Rb * e_3;

    zb_des = - (-Kp * err_pos -Kv * dot_err_pos - uav_params.mass * g * e_3 + uav_params.mass * acc_des) / (-Kp * err_pos -Kv * dot_err_pos - uav_params.mass * g * e_3 + uav_params.mass * acc_des).norm();

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "outer_loop");
    ros::NodeHandle nh;

    double Ts = 0.01;
    double t_iniz = 0.1;
    double ttot = 15;
    double tdead = 10;
    ros::Rate loopRate(1.0 / Ts);
    
    zb_des << 0,0,1;
    u_T = 0.0;
    
    ros::Subscriber imu_sub = nh.subscribe("/ardrone/ground_truth/imu", 10, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ardrone/ground_truth/odometry", 10, odomCallback);
  
    ros::Publisher psi_pub      = nh.advertise<std_msgs::Float64> ("/ardrone/geom_control/psi_des", 10);
    ros::Publisher zb_des_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/zb_des", 10);
    ros::Publisher u_T_pub       = nh.advertise<std_msgs::Float64> ("/ardrone/geom_control/u_T", 10);

    ros::Publisher err_p_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/err_p", 10);
    ros::Publisher err_v_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/err_v", 10);

    ros::Publisher q_pub        = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/q", 10);
    ros::Publisher dot_q_pub    = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/dot_q", 10);
    ros::Publisher ddot_q_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/ddot_q", 10);

    UAVParameters uav_params;
    if (!loadUAVParameters(uav_params)) {
        ROS_ERROR("Failed to load UAV parameters.");
        return -1;
    }

    InitialFinalConditions conditions;
    conditions.q0 << pose.position.x, pose.position.y, -0.08, 0;
    conditions.dot_q0 << 0, 0, 0, 0;
    conditions.ddot_q0 << 0, 0, 0, 0;
    conditions.dddot_q0 << 0, 0, 0, 0;

    conditions.qf << -2, -2, -5, 0;
    conditions.dot_qf << 0, 0, 0, 0;
    conditions.ddot_qf << 0, 0, 0, 0;
    conditions.dddot_qf << 0, 0, 0, 0;

    // Kp << 1, 0, 0,
    //       0, 1, 0,
    //       0, 0, 1;
    // Kv << 1, 0, 0,
    //       0, 1, 0,
    //       0, 0, 1;    

    // Kp << 10, 0, 0,
    //       0, 10, 0,
    //       0, 0, 10;
    // Kv << 4.7, 0, 0,
    //       0, 4.7, 0,
    //       0, 0, 4.7;  

    Kp << 6, 0, 0,
          0, 6, 0,
          0, 0, 6;
    Kv << 4.7, 0, 0,
          0, 4.7, 0,
          0, 0, 4.7;  


    Trajectory traj = generateTrajectory(conditions, t_iniz, ttot, tdead, Ts);

    Vector3d q (pose.position.x,pose.position.y,-pose.position.z);
    Vector3d dot_q(0,0,0);
    Vector3d ddot_q(0,0,0);

    int i_time = 0;
    float psi = 0;

        ROS_INFO_STREAM("INIZIO OUTER");

    while(ros::ok()){


        q << traj.q[i_time][0], traj.q[i_time][1], traj.q[i_time][2];
        dot_q << traj.dot_q[i_time][0], traj.dot_q[i_time][1], traj.dot_q[i_time][2];
        ddot_q << traj.ddot_q[i_time][0], traj.ddot_q[i_time][1], traj.ddot_q[i_time][2];

        q_pub.publish(EigenToVector3(q));
        dot_q_pub.publish(EigenToVector3(dot_q));
        ddot_q_pub.publish(EigenToVector3(ddot_q));

        psi = traj.q[i_time].tail<1>()[0];

        std_msgs::Float64 psi_msg;
        psi_msg.data = psi;
        psi_pub.publish(psi_msg);

        // OUTER LOOP
        outer_loop(uav_params, q, dot_q, ddot_q);
    
        geometry_msgs::Vector3 zb_des_msg;
        zb_des_msg = EigenToVector3(zb_des);

        if (!std::isnan(zb_des_msg.x) && !std::isnan(zb_des_msg.y) && !std::isnan(zb_des_msg.z)) {
            zb_des_pub.publish(zb_des_msg);
        }
            ROS_INFO_STREAM("OUTER zb_des: " << zb_des.transpose());


        std_msgs::Float64 u_T_msg;
        u_T_msg.data = u_T;
        if (!std::isnan(u_T_msg.data)){
            u_T_pub.publish(u_T_msg);
        }
            ROS_INFO_STREAM("OUTER u_T: " << u_T);

        
        geometry_msgs::Vector3 err_pos_msg = EigenToVector3(err_p);
        err_p_pub.publish(err_pos_msg);
         
        geometry_msgs::Vector3 dot_err_pos_msg = EigenToVector3(err_v);
        err_v_pub.publish(dot_err_pos_msg);



        i_time++;

        if (i_time == traj.q.size()){
            return 0;
        }

        ros::spinOnce();
        loopRate.sleep();

    }

    return 0;
}


void imuCallback(const sensor_msgs::Imu& msg) {

    ang_vel = Vec3MsgToEigen(msg.angular_velocity);
    Rb = (quaternionToRotation(msg.orientation));
    Rb = enuToNed(Rb);

}


void odomCallback(const nav_msgs::Odometry& msg) {

    pose = msg.pose.pose;
    twist = msg.twist.twist;
    //Rb = quaternionToRotation(pose.orientation);

}