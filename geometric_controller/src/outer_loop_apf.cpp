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
Vector3d F;

void apfCallback(const geometry_msgs::Vector3& msg);
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
    ros::init(argc, argv, "outer_loop_apf");
    ros::NodeHandle nh;

    double Ts = 0.01;
    double t_iniz = 0.1;
    double ttot = 15;
    double tdead = 10;
    ros::Rate loopRate(1.0 / Ts);
    
    zb_des << 0,0,1;
    u_T = 0.0;
    
    ros::Subscriber afp_sub = nh.subscribe("/ardrone/geom_controller/apf", 10, apfCallback);
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

    Kp << 6, 0, 0,
          0, 6, 0,
          0, 0, 6;
    Kv << 4.7, 0, 0,
          0, 4.7, 0,
          0, 0, 4.7;  

    Vector3d q (pose.position.x,pose.position.y,-pose.position.z);
    Vector3d dot_q(0,0,0);
    Vector3d ddot_q(0,0,0);

    float psi = 0;

    TrajectoryAPF trajPrev;

    trajPrev.q << pose.position.x,pose.position.y,-pose.position.z,0;
    trajPrev.dot_q << 0,0,0,0;
    trajPrev.ddot_q << 0,0,0,0;


    while(ros::ok()){

        TrajectoryAPF trajectory;

        // if (std::isnan(F.x) || std::isnan(F.y) || std::isnan(F.z)) {
        //     F << 0,0,0;
        // }
        
        trajectory = APFplanner (F, trajPrev, Ts);

        q       << trajectory.q[0], trajectory.q[1], trajectory.q[2]; 
        dot_q   << trajectory.dot_q[0], trajectory.dot_q[1], trajectory.dot_q[2];
        ddot_q  << trajectory.ddot_q[0], trajectory.ddot_q[1], trajectory.ddot_q[2];

        // PASSAGGIO VARIABILI ALL'INNER LOOP

        psi = trajectory.q[3];
        
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



        // SCRITTURA DATI PER BAG
        q_pub.publish(EigenToVector3(q));
        dot_q_pub.publish(EigenToVector3(dot_q));
        ddot_q_pub.publish(EigenToVector3(ddot_q));     

        geometry_msgs::Vector3 err_pos_msg = EigenToVector3(err_p);
        err_p_pub.publish(err_pos_msg);
         
        geometry_msgs::Vector3 dot_err_pos_msg = EigenToVector3(err_v);
        err_v_pub.publish(dot_err_pos_msg);



        ros::spinOnce();
        loopRate.sleep();

    }

    return 0;
}

void apfCallback (const geometry_msgs::Vector3& msg){
    F.x() = msg.x;
    F.y() = msg.y;
    F.z() = msg.z;
}

void imuCallback(const sensor_msgs::Imu& msg) {

    ang_vel = AngVelToEigen(msg.angular_velocity);
    Rb = (quaternionToRotation(msg.orientation));
    Rb = enuToNed(Rb);

}

void odomCallback(const nav_msgs::Odometry& msg) {

    pose = msg.pose.pose;
    twist = msg.twist.twist;
    //Rb = quaternionToRotation(pose.orientation);

}