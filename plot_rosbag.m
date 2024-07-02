clear all
close all
clc

bagfile = 'geom_controller.bag';

bag = rosbag(bagfile);

disp(bag.AvailableTopics);

bag_odometry = select(bag, 'Topic','/ardrone/odometry_sensor1/odometry');
bag_rpy_thrust = select(bag, 'Topic', '/ardrone/command/roll_pitch_yawrate_thrust1');
bag_psi_des = select(bag, 'Topic', '/ardrone/geom_control/psi_des');
bag_zb_des = select(bag, 'Topic', '/ardrone/geom_control/zb_des');
bag_err_p = select(bag, 'Topic', '/ardrone/geom_control/err_p');
bag_err_v = select(bag, 'Topic', '/ardrone/geom_control/err_v');
bag_err_R = select(bag, 'Topic', '/ardrone/geom_control/err_R');
bag_err_W = select(bag, 'Topic', '/ardrone/geom_control/err_W');

bag_q = select(bag, 'Topic', '/ardrone/geom_control/q');
bag_dot_q = select(bag, 'Topic', '/ardrone/geom_control/dot_q');
bag_ddot_q = select(bag, 'Topic', '/ardrone/geom_control/ddot_q');

bag_omega_bb = select(bag, 'Topic', '/ardrone/geom_control/omega_bb');
bag_Rb_des = select(bag, 'Topic', '/ardrone/geom_control/Rb_des');
bag_omega_bb_des = select(bag, 'Topic', '/ardrone/geom_control/omega_bb_des');
bag_dot_Rb_des = select(bag, 'Topic', '/ardrone/geom_control/dot_Rb_des');
bag_dot_omega_bb_des = select(bag, 'Topic', '/ardrone/geom_control/dot_omega_bb_des');

bag_comm_speed = select(bag, 'Topic', '/ardrone/command/motor_speed');
bag_read_speed = select(bag, 'Topic', '/ardrone/motor_speed');

bag_gt_odometry = select(bag, 'Topic','/ardrone/ground_truth/odometry');
bag_gt_imu = select(bag, 'Topic','/ardrone/ground_truth/imu');
bag_apf = select(bag, 'Topic','/ardrone/geom_controller/apf');



%% Leggi tutti i messaggi dai topic selezionati

pos = timeseries(bag_odometry,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z');
ori = timeseries(bag_odometry,'Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
vel = timeseries(bag_odometry,'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Linear.Z');
ang_vel = timeseries(bag_odometry,'Twist.Twist.Angular.X','Twist.Twist.Angular.Y','Twist.Twist.Angular.Z');                                                                                                                                                                                             

imu_ori = timeseries(bag_gt_imu,'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W');
imu_acc = timeseries(bag_gt_imu,'LinearAcceleration.X','LinearAcceleration.Y','LinearAcceleration.Z');
imu_ang_vel = timeseries(bag_gt_imu,'AngularVelocity.X','AngularVelocity.Y','AngularVelocity.Z');

q = timeseries(bag_q,'X','Y','Z');
dot_q = timeseries(bag_dot_q,'X','Y','Z');
ddot_q = timeseries(bag_ddot_q,'X','Y','Z');

psi_des = timeseries(bag_psi_des,'Data');
zb_des = timeseries(bag_zb_des,'X','Y','Z');
err_p = timeseries(bag_err_p,'X','Y','Z');
err_v = timeseries(bag_err_v,'X','Y','Z');
err_R = timeseries(bag_err_R,'X','Y','Z');
err_W = timeseries(bag_err_W,'X','Y','Z');

omega_bb = timeseries(bag_omega_bb,'X','Y','Z');
Rb_des = timeseries(bag_Rb_des,'X','Y','Z','W');
omega_bb_des = timeseries(bag_omega_bb_des,'X','Y','Z');
dot_Rb_des = timeseries(bag_dot_Rb_des,'X','Y','Z','W');
dot_omega_bb_des = timeseries(bag_dot_omega_bb_des,'X','Y','Z');

comm_speed_msgs = readMessages(bag_comm_speed, 'DataFormat', 'struct');
read_speed_msgs = readMessages(bag_read_speed, 'DataFormat', 'struct');

comm_speed1 = cellfun(@(m) m.AngularVelocities(1), comm_speed_msgs);
comm_speed2 = cellfun(@(m) m.AngularVelocities(2), comm_speed_msgs);
comm_speed3 = cellfun(@(m) m.AngularVelocities(3), comm_speed_msgs);
comm_speed4 = cellfun(@(m) m.AngularVelocities(4), comm_speed_msgs);

comm_speed = [comm_speed1 comm_speed2 comm_speed3 comm_speed4];

read_speed1 = cellfun(@(m) m.AngularVelocities(1), read_speed_msgs);
read_speed2 = cellfun(@(m) m.AngularVelocities(2), read_speed_msgs);
read_speed3 = cellfun(@(m) m.AngularVelocities(3), read_speed_msgs);
read_speed4 = cellfun(@(m) m.AngularVelocities(4), read_speed_msgs);

read_speed = [read_speed1 read_speed2 read_speed3 read_speed4];

rpy_thrust_msgs = readMessages(bag_rpy_thrust, 'DataFormat', 'struct');
roll = cellfun(@(m) m.Roll, rpy_thrust_msgs);
pitch = cellfun(@(m) m.Pitch, rpy_thrust_msgs);
yaw_rate = cellfun(@(m) m.YawRate, rpy_thrust_msgs);

tau_b = [roll pitch yaw_rate];
thrust = cellfun(@(m) m.Thrust.Z, rpy_thrust_msgs);

u_T = thrust;

F = timeseries(bag_apf,'X','Y','Z');


%% PLOT 
legend_vec_xyz = {'x', 'y', 'z'};
legend_vec_rpy = {'roll', 'pitch', 'yaw'};
legend_vec_quat = {'x', 'y', 'z', 'w'};

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(3,1,1)
plot_ts(q,'time [sec]','position[m]',"Traj Pos",legend_vec_xyz);
subplot(3,1,2)
plot_ts(dot_q,'time [sec]','velocity[m/s]',"Traj Vel",legend_vec_xyz);
subplot(3,1,3)
plot_ts(ddot_q,'time [sec]','acceleration[m/s^2]',"Traj Acc",legend_vec_xyz);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(pos,'time [sec]','position [m]',"Position",legend_vec_xyz);
subplot(2,2,2)
plot_ts(ori,'time [sec]','quaternion',"Orientation",legend_vec_quat);
subplot(2,2,3)
plot_ts(vel,'time [sec]','velocity[m/s]',"Linear Velocity",legend_vec_xyz);
subplot(2,2,4)
plot_ts(ang_vel,'time [sec]','quaternion',"Angular Velocity",legend_vec_quat);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(imu_acc,'time [sec]','position [m]',"IMU ACC",legend_vec_xyz);
subplot(2,2,2)
plot_ts(imu_ori,'time [sec]','quaternion',"IMU Orientation",legend_vec_quat);
subplot(2,2,3)
plot_ts(imu_ang_vel,'time [sec]','quaternion',"IMU Angular Velocity",legend_vec_quat);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(err_p,'time [sec]','error pos [m]',"Error P",legend_vec_xyz);
subplot(2,2,2)
plot_ts(err_v,'time [sec]','error vel [m/s]',"Error V",legend_vec_xyz);
subplot(2,2,3)
plot_ts(err_R,'time [sec]','error rot [rad]',"Error R",legend_vec_xyz);
subplot(2,2,4)
plot_ts(err_W,'time [sec]','error ang vel [rad/s]',"Error W ",legend_vec_xyz);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(3,1,1)
plot_ts(omega_bb,'time [sec]','omega_bb[rad/s]',"omega_bb",legend_vec_xyz);
subplot(3,1,2)
plot_ts(omega_bb_des,'time [sec]','omega_bb_des[rad/s]',"omega_bb_des",legend_vec_xyz);
subplot(3,1,3)
plot_ts(dot_omega_bb_des,'time [sec]','dot_omega_bb_des[rad/s^2]',"dot_omega_bb_des",legend_vec_xyz);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(3,1,1)
plot_ts(Rb_des,'time [sec]','Rb_des[rad]',"Rb_des",legend_vec_quat);
subplot(3,1,2)
plot_ts(dot_Rb_des,'time [sec]','dot_Rb_des[rad/s]',"dot_Rb_des",legend_vec_quat);
subplot(3,1,3)
plot_ts(zb_des,'time [sec]','vector',"Zb_des",legend_vec_xyz)


figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
singleplot(tau_b, 'time [sec]', 'command torque', 'tau_b', legend_vec_rpy)

subplot(2,2,2)
singleplot(u_T, 'time [sec]', 'command thrust', 'u_T', 'u_T')

subplot(2,2,3)
singleplot(read_speed, 'time [sec]', 'motor speed [rpm]', 'Read Motor Speed', {'m0','m1','m2','m3'})

subplot(2,2,4)
singleplot(comm_speed, 'time [sec]', 'motor speed [rpm]', 'Commanded Motor Speed', {'m0','m1','m2','m3'})

figure()
plot_ts(F,'time [sec]','F_tot[N]',"APF Total Force",legend_vec_xyz)



