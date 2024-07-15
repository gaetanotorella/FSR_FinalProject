#include "geometric_controller/utils_fun.h"

#define g 9.81

geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
Matrix3d Rb;
Vector3d ang_vel;
float u_T;
Vector3d tau_b;

void odomCallback(const nav_msgs::Odometry& msg);
void imuCallback(const sensor_msgs::Imu& msg);
void commandCallback(const mav_msgs::RollPitchYawrateThrust& msg);

Matrix3d C_fun(Vector3d prev_eta_b, Vector3d prev_eta_b_dot, UAVParameters uav_params);
Matrix3d Q_dot_fun(Vector3d prev_eta_b, Vector3d prev_eta_b_dot);
Matrix3d Q_fun(Vector3d prev_eta_b);
Matrix3d R_fun(Vector3d prev_eta_b);
Matrix3d M_fun(Vector3d eta_b, UAVParameters uav_params);

void run_estimator(UAVParameters uav_params, double Ts, int order, VectorXd& coeff, VectorXd& q, VectorXd& ext_wrench, VectorXd& gamma, Vector3d eta_b, Vector3d p_b_dot, Vector3d eta_b_dot, float u_T, Vector3d tau_b, Vector3d prev_eta_b, Vector3d prev_p_b_dot, Vector3d prev_eta_b_dot, VectorXd& prev_gamma);


int main(int argc, char** argv) {

    ros::init(argc, argv, "momentum_estimator");
    ros::NodeHandle nh;

    double Ts = 0.01;

    ros::Rate loopRate(1.0 / Ts);
    
    // ros::Publisher afp_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/geom_controller/apf", 10);
    // ros::Publisher F_rep_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/geom_controller/F_rep", 10);
    // ros::Publisher dist_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/geom_controller/dist", 10);

    ros::Subscriber imu_sub = nh.subscribe("/ardrone/ground_truth/imu", 10, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ardrone/ground_truth/odometry", 10, odomCallback);
    ros::Subscriber command_sub = nh.subscribe("/ardrone/command/roll_pitch_yawrate_thrust", 10, commandCallback);

    ros::Publisher f_e_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/momentum_estimator/f_e", 10);
    ros::Publisher tau_e_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/momentum_estimator/tau_e", 10);


    UAVParameters uav_params;
    if (!loadUAVParameters(uav_params)) {
        ROS_ERROR("Failed to load UAV parameters.");
        return -1;
    }

    int order = 1;
    VectorXd coeff(1);
    VectorXd q(6); 
    VectorXd ext_wrench(6);
    VectorXd gamma(6); 

    coeff << 8;
    q << 0,0,0,0,0,0;
    ext_wrench << 0,0,0,0,0,0;
    gamma << 0,0,0,0,0,0;

    Vector3d eta_b;                 // orientation RPY
    Vector3d p_b_dot;               // linear velocity
    Vector3d eta_b_dot;             // angular velocity

    Vector3d prev_eta_b;
    Vector3d prev_p_b_dot;
    Vector3d prev_eta_b_dot;
    VectorXd prev_gamma(6); 

    prev_eta_b << 0,0,0;
    prev_p_b_dot << 0,0,0;
    prev_eta_b_dot << 0,0,0;
    prev_gamma << 0,0,0,0,0,0;


    while(ros::ok()){
        
        eta_b = Rb.eulerAngles(0,1,2);
        p_b_dot = TwistLinearToEigen(twist);
        eta_b_dot = ang_vel;

        run_estimator(uav_params, Ts, order, coeff, q, ext_wrench, gamma, eta_b, p_b_dot, eta_b_dot, u_T, tau_b, prev_eta_b, prev_p_b_dot, prev_eta_b_dot, prev_gamma);

        prev_eta_b = eta_b;
        prev_p_b_dot = p_b_dot;
        prev_eta_b_dot = eta_b_dot;

        Vector3d temp;

        temp.x() = ext_wrench[0];
        temp.y() = ext_wrench[1];
        temp.z() = ext_wrench[2];
        f_e_pub.publish(EigenToVector3(temp));
        Vector3d temp1;
        temp1.x() = ext_wrench[3];
        temp1.y() = ext_wrench[4];
        temp1.z() = ext_wrench[5];
        tau_e_pub.publish(EigenToVector3(temp1));

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

void commandCallback(const mav_msgs::RollPitchYawrateThrust& msg){

    tau_b[0] = msg.roll;
    tau_b[1] = msg.pitch;
    tau_b[2] = msg.yaw_rate;
    u_T = msg.thrust.z;

}


void run_estimator(UAVParameters uav_params, double Ts, int order, VectorXd& coeff, VectorXd& q, VectorXd& ext_wrench, VectorXd& gamma, Vector3d eta_b, Vector3d p_b_dot, Vector3d eta_b_dot, float u_T, Vector3d tau_b, Vector3d prev_eta_b, Vector3d prev_p_b_dot, Vector3d prev_eta_b_dot, VectorXd& prev_gamma){
    
    VectorXd q_k_plus_1(6);
    q_k_plus_1 << 0,0,0,0,0,0;
    
    Matrix3d M_eye = Matrix3d::Identity();
    Matrix3d M_zero = Matrix3d::Zero();
    Vector3d e_3 = Vector3d(0, 0, 1);

    MatrixXd M_tot = MatrixXd::Zero(6,6);    
    M_tot << uav_params.mass * M_eye, M_zero,
                  M_zero, M_fun(eta_b, uav_params);

    VectorXd state_dot(6);
    state_dot << p_b_dot, eta_b_dot;

    q_k_plus_1 = M_tot * state_dot;

    Matrix3d C = C_fun(prev_eta_b, prev_eta_b_dot, uav_params);
    Matrix3d Q = Q_fun(prev_eta_b);
    Matrix3d Rb = R_fun(prev_eta_b);

    Vector3d vec1 = Vector3d::Zero();
    Vector3d vec2 = Vector3d::Zero();

    vec1 << uav_params.mass * g * e_3 - u_T * Rb * e_3;
    vec2 << C.transpose() * prev_eta_b_dot + Q.transpose() * tau_b;

    VectorXd vec_tot(6);
    vec_tot << vec1, vec2;

    gamma = prev_gamma + coeff[0] * ((q_k_plus_1 - q) - Ts * vec_tot - Ts * ext_wrench);
    //gamma = prev_gamma + coeff[0] * ((q_k_plus_1 - q) - Ts * (uav_params.mass * g * e_3 - u_T * Rb * e_3, C.transpose() * prev_eta_b_dot + Q.transpose() * tau_b) - Ts * ext_wrench);


// da implementare
    //gamma = update_gamma(gamma, k, order, Ts, coeff, ext_wrench);

    ext_wrench = gamma;
    q = q_k_plus_1;
    prev_gamma = gamma;

}

Matrix3d C_fun(Vector3d prev_eta_b, Vector3d prev_eta_b_dot, UAVParameters uav_params){
    
    Matrix3d C;
    Matrix3d Q;
    Matrix3d Q_dot;
    Matrix3d S;

    Q = Q_fun(prev_eta_b);
    S = skew(Q * prev_eta_b_dot);
    Q_dot = Q_dot_fun(prev_eta_b, prev_eta_b_dot);
    C = Q.transpose() * S * uav_params.Ib * Q + Q.transpose() * uav_params.Ib * Q_dot;

    return C;
}

Matrix3d Q_fun(Vector3d prev_eta_b){

    Matrix3d Q;
    Q << 1, 0, -sin(prev_eta_b.y()),
        0, cos(prev_eta_b.x()), cos(prev_eta_b.y())*sin(prev_eta_b.x()),
        0, -sin(prev_eta_b.x()), cos(prev_eta_b.x())*cos(prev_eta_b.y());

    return Q;

}

Matrix3d R_fun(Vector3d prev_eta_b){

    Matrix3d R;
    Eigen::AngleAxisd rollAngle(prev_eta_b.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(prev_eta_b.y(), Eigen::Vector3d::UnitY()); 
    Eigen::AngleAxisd yawAngle(prev_eta_b.z(), Eigen::Vector3d::UnitZ());  

    R = yawAngle * pitchAngle * rollAngle; 

    return R;
}

Matrix3d Q_dot_fun(Vector3d prev_eta_b, Vector3d prev_eta_b_dot){

    double roll = prev_eta_b.x();
    double pitch = prev_eta_b.y();
    double yaw = prev_eta_b.z();

    double roll_dot = prev_eta_b_dot.x();
    double pitch_dot = prev_eta_b_dot.y();
    double yaw_dot = prev_eta_b_dot.z();

    Eigen::Matrix3d Q_dot;
    Q_dot << 0, 0, -std::cos(pitch) * pitch_dot,
             0, -std::sin(roll) * roll_dot, -std::sin(pitch) * std::sin(roll) * pitch_dot + std::cos(pitch) * std::cos(roll) * roll_dot,
             0, -std::cos(roll) * roll_dot, -std::sin(pitch) * std::cos(roll) * pitch_dot - std::cos(pitch) * std::sin(roll) * roll_dot;
    
    return Q_dot;
}

Matrix3d M_fun(Vector3d eta_b, UAVParameters uav_params){

    Matrix3d M;
    Matrix3d Q;
    Q = Q_fun(eta_b);

    M = Q.transpose() * uav_params.Ib * Q;

    return M;
}