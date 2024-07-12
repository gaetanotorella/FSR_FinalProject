#include "geometric_controller/utils_fun.h"

// EIGEN FUNCTIONS

geometry_msgs::Vector3 EigenToVector3 (const Eigen::Vector3d& entinty){
    geometry_msgs::Vector3 vec;
    vec.x = entinty[0];
    vec.y = entinty[1];
    vec.z = entinty[2];
    return vec;
}

Eigen::Vector3d PoseToEigen(const geometry_msgs::Pose& pose) {
    Eigen::Vector3d vec;
    vec.x() = -pose.position.x;
    vec.y() = -pose.position.y;
    vec.z() = -pose.position.z;
    return vec;
}

Eigen::Vector3d AngVelToEigen(const geometry_msgs::Vector3& ang_vel) {
    Eigen::Vector3d vec;
    //todo
    vec.x() = ang_vel.x;
    vec.y() = ang_vel.y;
    vec.z() = ang_vel.z;
    return vec;
}

Eigen::Vector3d TwistLinearToEigen(const geometry_msgs::Twist& twist) {
    Eigen::Vector3d vec;
    vec.x() = -twist.linear.x;
    vec.y() = -twist.linear.y;
    vec.z() = -twist.linear.z;
    return vec;
}

Eigen::Vector3d TwistAngularToEigen(const geometry_msgs::Twist& twist) {
    Eigen::Vector3d vec;
    vec.x() = twist.angular.x;
    vec.y() = twist.angular.y;
    vec.z() = twist.angular.z;
    return vec;
}

Eigen::Matrix3d enuToNed(const Eigen::Matrix3d& rotationMatrixENU) {
    Eigen::Matrix3d T;
    T << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    return T * rotationMatrixENU * T.transpose();
}

Eigen::Matrix3d quaternionToRotation(const geometry_msgs::Quaternion& quaternion) {
    Eigen::Quaterniond quat(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    return quat.normalized().toRotationMatrix();
}

Eigen::Matrix3d skew(const Eigen::Vector3d& t){
    Eigen::Matrix3d t_hat;
    t_hat <<     0, -t[2],  t[1],
              t[2],     0, -t[0],
             -t[1],  t[0],     0;

    return t_hat;
}

Eigen::Vector3d wedge(const Eigen::Matrix3d& A){
    Eigen::Vector3d vec;
    vec << A(2,1), A(0,2), A(1,0);
    return vec;
}


// GET UAV PARAMETERS

bool loadUAVParameters(UAVParameters &uav_params) {
    ros::NodeHandle nh("~");

    if (!nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/mass", uav_params.mass)) {
        ROS_ERROR("Failed to get param 'mass'");
        return false;
    }

    uav_params.mass = uav_params.mass;

    double Ixx, Ixy, Ixz, Iyy, Iyz, Izz;
    if (!nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/inertia/xx", Ixx) ||
        !nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/inertia/xy", Ixy) ||
        !nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/inertia/xz", Ixz) ||
        !nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/inertia/yy", Iyy) ||
        !nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/inertia/yz", Iyz) ||
        !nh.getParam("/ardrone/roll_pitch_yawrate_thrust_controller_node/inertia/zz", Izz)) {
        ROS_ERROR("Failed to get inertia parameters");
        return false;
    }

    uav_params.Ib << Ixx, 0, 0,
                     0, Iyy, 0,
                     0, 0, Izz;

    return true;
}

#define arm_length 0.09
#define rotor_moment_constant 0.016
#define rotor_force_constant 8.54858e-06
#define angle0 -0.78539
#define angle1 2.35619
#define angle2 0.78539
#define angle3 -2.35619
#define direction0 1.0
#define direction1 1.0
#define direction2 -1.0
#define direction3 -1.0

Eigen::Matrix4d computeAllocationMatrix(){
    Eigen::Matrix4d alloc_mat;

    float l = arm_length;
    float rfc = rotor_force_constant;
    float rmc = rotor_moment_constant;

    alloc_mat <<    rotor_force_constant,   rotor_force_constant,   rotor_force_constant,   rotor_force_constant,
                        sin(angle0)*l*rfc,       sin(angle1)*l*rfc,       sin(angle2)*l*rfc,       sin(angle3)*l*rfc,
                        cos(angle0)*l*rfc,       cos(angle1)*l*rfc,       cos(angle3)*l*rfc,       cos(angle3)*l*rfc,
                                -rfc*rmc,                -rfc*rmc,                rfc*rmc,                rfc*rmc;

    return alloc_mat;
}


// TRAJECTORY GENERATOR

Trajectory generateTrajectory(const InitialFinalConditions& conditions, double t_iniz, double ttot, double tdead, double Ts) {
    int n = static_cast<int>(ttot / Ts);
    int total_n = n + static_cast<int>(tdead / Ts);

    Trajectory traj;
    traj.q.resize(n, Vector4d::Zero());
    traj.dot_q.resize(n, Vector4d::Zero());
    traj.ddot_q.resize(n, Vector4d::Zero());

    Vector4d a0, a1, a2, a3, a4, a5, a6, a7;

    for (int j = 0; j < 4; j++) {
        Matrix<double, 8, 8> A;
        A << pow(t_iniz, 7), pow(t_iniz, 6), pow(t_iniz, 5), pow(t_iniz, 4), pow(t_iniz, 3), pow(t_iniz, 2), t_iniz, 1,
             pow(ttot, 7), pow(ttot, 6), pow(ttot, 5), pow(ttot, 4), pow(ttot, 3), pow(ttot, 2), ttot, 1,
             7 * pow(t_iniz, 6), 6 * pow(t_iniz, 5), 5 * pow(t_iniz, 4), 4 * pow(t_iniz, 3), 3 * pow(t_iniz, 2), 2 * t_iniz, 1, 0,
             7 * pow(ttot, 6), 6 * pow(ttot, 5), 5 * pow(ttot, 4), 4 * pow(ttot, 3), 3 * pow(ttot, 2), 2 * ttot, 1, 0,
             42 * pow(t_iniz, 5), 30 * pow(t_iniz, 4), 20 * pow(t_iniz, 3), 12 * pow(t_iniz, 2), 6 * t_iniz, 2, 0, 0,
             42 * pow(ttot, 5), 30 * pow(ttot, 4), 20 * pow(ttot, 3), 12 * pow(ttot, 2), 6 * ttot, 2, 0, 0,
             210 * pow(t_iniz, 4), 120 * pow(t_iniz, 3), 60 * pow(t_iniz, 2), 24 * t_iniz, 6, 0, 0, 0,
             210 * pow(ttot, 4), 120 * pow(ttot, 3), 60 * pow(ttot, 2), 24 * ttot, 6, 0, 0, 0;

        Matrix<double, 8, 1> b;
        b << conditions.q0(j), conditions.qf(j), conditions.dot_q0(j), conditions.dot_qf(j),
             conditions.ddot_q0(j), conditions.ddot_qf(j), conditions.dddot_q0(j), conditions.dddot_qf(j);

        Matrix<double, 8, 1> a_temp = A.fullPivLu().solve(b);

        a7(j) = a_temp(0);
        a6(j) = a_temp(1);
        a5(j) = a_temp(2);
        a4(j) = a_temp(3);
        a3(j) = a_temp(4);
        a2(j) = a_temp(5);
        a1(j) = a_temp(6);
        a0(j) = a_temp(7);
    }

    for (int i = 0; i < n; i++) {
        double t = i * Ts;
        for (int j = 0; j < 4; j++) {
            traj.q[i][j] = a7(j) * pow(t, 7) + a6(j) * pow(t, 6) + a5(j) * pow(t, 5) + a4(j) * pow(t, 4) + a3(j) * pow(t, 3) + a2(j) * pow(t, 2) + a1(j) * t + a0(j);
            traj.dot_q[i][j] = 7 * a7(j) * pow(t, 6) + 6 * a6(j) * pow(t, 5) + 5 * a5(j) * pow(t, 4) + 4 * a4(j) * pow(t, 3) + 3 * a3(j) * pow(t, 2) + 2 * a2(j) * t + a1(j);
            traj.ddot_q[i][j] = 42 * a7(j) * pow(t, 5) + 30 * a6(j) * pow(t, 4) + 20 * a5(j) * pow(t, 3) + 12 * a4(j) * pow(t, 2) + 6 * a3(j) * t + 2 * a2(j);

        }
    }

    // Aggiunta del tempo di dead time

        traj.q.resize(total_n, traj.q[n-1]);
        traj.dot_q.resize(total_n, traj.dot_q[n-1]);
        traj.ddot_q.resize(total_n, traj.ddot_q[n-1]);
    
    ROS_INFO("fine planner");
    
    return traj;
}


// ARTIFICIAL POTENTIAL FIELD METHOD

TrajectoryAPF APFplanner (Vector3d F, TrajectoryAPF& trajPrev, double Ts){

    TrajectoryAPF state;
    state.q << 0,0,0,0;
    state.dot_q << 0,0,0,0;
    state.ddot_q << 0,0,0,0;

    // compute position
    state.q[0] = trajPrev.q[0] + F[0] * Ts;
    state.q[1] = trajPrev.q[1] + F[1] * Ts;
    state.q[2] = trajPrev.q[2] + F[2] * Ts;

    // compute velocity
    state.dot_q[0] = F[0];
    state.dot_q[1] = F[1];
    state.dot_q[2] = F[2];

    // compute acceleration
    state.ddot_q[0] =  (F[0] - trajPrev.dot_q[0]) / Ts;
    state.ddot_q[1] =  (F[1] - trajPrev.dot_q[1]) / Ts;
    state.ddot_q[2] =  (F[2] - trajPrev.dot_q[2]) / Ts;

    // update previous values
    trajPrev.q[0] = state.q[0];
    trajPrev.q[1] = state.q[1];
    trajPrev.q[2] = state.q[2];

    trajPrev.dot_q[0] = state.dot_q[0];
    trajPrev.dot_q[1] = state.dot_q[1];
    trajPrev.dot_q[2] = state.dot_q[2];

    return state;

} 


bool loadGoal(Vector3d& goal) {
    ros::NodeHandle nh("~");

    double x,y,z;
    if (!nh.getParam("/x_goal", x) ||
        !nh.getParam("/y_goal", y) ||
        !nh.getParam("/z_goal", z)) {
        ROS_ERROR("Failed to get param 'goal'");
        return false;
    }

    goal << x,y,z;
    return true;
}
