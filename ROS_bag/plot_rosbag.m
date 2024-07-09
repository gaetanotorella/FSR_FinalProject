clear all
close all
clc

bagfile = 'geom_controller.bag';
bag = rosbag(bagfile);
disp(bag.AvailableTopics);


bag_odometry = select(bag, 'Topic','/ardrone/odometry_sensor1/odometry');
bag_rpy_thrust = select(bag, 'Topic', '/ardrone/command/roll_pitch_yawrate_thrust1');
bag_psi_des = select(bag, 'Topic', '/ardrone/geom_control/psi_des');
bag_err_p = select(bag, 'Topic', '/ardrone/geom_control/err_p');
bag_err_v = select(bag, 'Topic', '/ardrone/geom_control/err_v');
bag_err_R = select(bag, 'Topic', '/ardrone/geom_control/err_R');
bag_err_W = select(bag, 'Topic', '/ardrone/geom_control/err_W');

bag_q = select(bag, 'Topic', '/ardrone/geom_control/q');
bag_dot_q = select(bag, 'Topic', '/ardrone/geom_control/dot_q');
bag_ddot_q = select(bag, 'Topic', '/ardrone/geom_control/ddot_q');

% bag_omega_bb = select(bag, 'Topic', '/ardrone/geom_control/omega_bb');
% bag_Rb_des = select(bag, 'Topic', '/ardrone/geom_control/Rb_des');
% bag_omega_bb_des = select(bag, 'Topic', '/ardrone/geom_control/omega_bb_des');
% bag_dot_Rb_des = select(bag, 'Topic', '/ardrone/geom_control/dot_Rb_des');
% bag_dot_omega_bb_des = select(bag, 'Topic', '/ardrone/geom_control/dot_omega_bb_des');
% bag_zb_des = select(bag, 'Topic', '/ardrone/geom_control/zb_des');

bag_comm_speed = select(bag, 'Topic', '/ardrone/command/motor_speed');
bag_read_speed = select(bag, 'Topic', '/ardrone/motor_speed');

bag_gt_odometry = select(bag, 'Topic','/ardrone/ground_truth/odometry');
bag_gt_imu = select(bag, 'Topic','/ardrone/ground_truth/imu');
bag_apf = select(bag, 'Topic','/ardrone/geom_controller/apf');
bag_F_rep = select(bag, 'Topic','/ardrone/geom_controller/F_rep');
% bag_dist3d = select(bag, 'Topic','/ardrone/geom_controller/dist');



%% Leggi tutti i messaggi dai topic selezionati

pos = timeseries(bag_odometry,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z');
ori = timeseries(bag_odometry,'Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
vel = timeseries(bag_odometry,'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Linear.Z');
ang_vel = timeseries(bag_odometry,'Twist.Twist.Angular.X','Twist.Twist.Angular.Y','Twist.Twist.Angular.Z');                                                                                                                                                                                             

% imu_ori = timeseries(bag_gt_imu,'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W');
% imu_acc = timeseries(bag_gt_imu,'LinearAcceleration.X','LinearAcceleration.Y','LinearAcceleration.Z');
% imu_ang_vel = timeseries(bag_gt_imu,'AngularVelocity.X','AngularVelocity.Y','AngularVelocity.Z');

q = timeseries(bag_q,'X','Y','Z');
dot_q = timeseries(bag_dot_q,'X','Y','Z');
ddot_q = timeseries(bag_ddot_q,'X','Y','Z');

psi_des = timeseries(bag_psi_des,'Data');
err_p = timeseries(bag_err_p,'X','Y','Z');
err_v = timeseries(bag_err_v,'X','Y','Z');
err_R = timeseries(bag_err_R,'X','Y','Z');
err_W = timeseries(bag_err_W,'X','Y','Z');

% omega_bb = timeseries(bag_omega_bb,'X','Y','Z');
% Rb_des = timeseries(bag_Rb_des,'X','Y','Z','W');
% omega_bb_des = timeseries(bag_omega_bb_des,'X','Y','Z');
% dot_Rb_des = timeseries(bag_dot_Rb_des,'X','Y','Z','W');
% dot_omega_bb_des = timeseries(bag_dot_omega_bb_des,'X','Y','Z');
% zb_des = timeseries(bag_zb_des,'X','Y','Z');

comm_speed_msgs = readMessages(bag_comm_speed, 'DataFormat', 'struct');

comm_speed1 = cellfun(@(m) m.AngularVelocities(1), comm_speed_msgs);
comm_speed2 = cellfun(@(m) m.AngularVelocities(2), comm_speed_msgs);
comm_speed3 = cellfun(@(m) m.AngularVelocities(3), comm_speed_msgs);
comm_speed4 = cellfun(@(m) m.AngularVelocities(4), comm_speed_msgs);

comm_speed = [comm_speed1 comm_speed2 comm_speed3 comm_speed4];
comm_speed_time = linspace(bag_comm_speed.StartTime,bag_comm_speed.EndTime,size(bag_comm_speed.MessageList,1));
cs = timeseries(comm_speed,comm_speed_time);
comm_speed = cs;


read_speed_msgs = readMessages(bag_read_speed, 'DataFormat', 'struct');

read_speed1 = cellfun(@(m) m.AngularVelocities(1), read_speed_msgs);
read_speed2 = cellfun(@(m) m.AngularVelocities(2), read_speed_msgs);
read_speed3 = cellfun(@(m) m.AngularVelocities(3), read_speed_msgs);
read_speed4 = cellfun(@(m) m.AngularVelocities(4), read_speed_msgs);

read_speed = [read_speed1 read_speed2 read_speed3 read_speed4];
read_speed_time = linspace(bag_read_speed.StartTime,bag_read_speed.EndTime,size(bag_read_speed.MessageList,1));
rs = timeseries(read_speed,read_speed_time);
read_speed = rs;

rpy_thrust_msgs = readMessages(bag_rpy_thrust, 'DataFormat', 'struct');
roll = cellfun(@(m) m.Roll, rpy_thrust_msgs);
pitch = cellfun(@(m) m.Pitch, rpy_thrust_msgs);
yaw_rate = cellfun(@(m) m.YawRate, rpy_thrust_msgs);

tau_b = [roll pitch yaw_rate];

tau_b_time = linspace(bag_rpy_thrust.StartTime,bag_rpy_thrust.EndTime,size(bag_rpy_thrust.MessageList,1));
tau_bs = timeseries(tau_b,tau_b_time);
tau_b = tau_bs;

thrust = cellfun(@(m) m.Thrust.Z, rpy_thrust_msgs);
thrusts = timeseries(thrust,tau_b_time);
u_T = thrusts;


F = timeseries(bag_apf,'X','Y','Z');
F_rep = timeseries(bag_F_rep,'X','Y','Z');
% dist3d = timeseries(bag_dist3d,'X','Y','Z');


%% PLOT 
legend_vec_xyz = {'x', 'y', 'z'};
legend_vec_rpy = {'roll', 'pitch', 'yaw'};
legend_vec_quat = {'x', 'y', 'z', 'w'};

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(3,1,1)
q.Data = -q.Data;
plot_ts(q,'time [sec]','position[m]',"Desired Position",legend_vec_xyz);
subplot(3,1,2)
plot_ts(dot_q,'time [sec]','velocity[m/s]',"Desried Velocity",legend_vec_xyz);
subplot(3,1,3)
plot_ts(ddot_q,'time [sec]','acceleration[m/s^2]',"Desried Acceleration",legend_vec_xyz);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(pos,'time [sec]','position [m]',"Position",legend_vec_xyz);
subplot(2,2,2)
plot_ts(ori,'time [sec]','quaternion',"Orientation",legend_vec_quat);
subplot(2,2,3)
plot_ts(vel,'time [sec]','velocity[m/s]',"Linear Velocity",legend_vec_xyz);
subplot(2,2,4)
plot_ts(ang_vel,'time [sec]','quaternion',"Angular Velocity",legend_vec_quat);

% figure('Renderer', 'painters', 'Position', [1 73 1440 724])
% subplot(2,2,1)
% plot_ts(imu_acc,'time [sec]','position [m]',"IMU ACC",legend_vec_xyz);
% subplot(2,2,2)
% plot_ts(imu_ori,'time [sec]','quaternion',"IMU Orientation",legend_vec_quat);
% subplot(2,2,3)
% plot_ts(imu_ang_vel,'time [sec]','quaternion',"IMU Angular Velocity",legend_vec_quat);

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(err_p,'time [sec]','error pos [m]',"Error P",legend_vec_xyz);
subplot(2,2,2)
plot_ts(err_v,'time [sec]','error vel [m/s]',"Error V",legend_vec_xyz);
subplot(2,2,3)
plot_ts(err_R,'time [sec]','error rot [rad]',"Error R",legend_vec_xyz);
subplot(2,2,4)
plot_ts(err_W,'time [sec]','error ang vel [rad/s]',"Error W ",legend_vec_xyz);

% figure('Renderer', 'painters', 'Position', [1 73 1440 724])
% subplot(3,1,1)
% plot_ts(omega_bb,'time [sec]','omega_bb[rad/s]',"omega_bb",legend_vec_xyz);
% subplot(3,1,2)
% plot_ts(omega_bb_des,'time [sec]','omega_bb_des[rad/s]',"omega_bb_des",legend_vec_xyz);
% subplot(3,1,3)
% plot_ts(dot_omega_bb_des,'time [sec]','dot_omega_bb_des[rad/s^2]',"dot_omega_bb_des",legend_vec_xyz);

% figure('Renderer', 'painters', 'Position', [1 73 1440 724])
% subplot(3,1,1)
% plot_ts(Rb_des,'time [sec]','Rb_des[rad]',"Rb_des",legend_vec_quat);
% subplot(3,1,2)
% plot_ts(dot_Rb_des,'time [sec]','dot_Rb_des[rad/s]',"dot_Rb_des",legend_vec_quat);
% subplot(3,1,3)
% plot_ts(zb_des,'time [sec]','vector',"Zb_des",legend_vec_xyz)


figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(tau_b, 'time [sec]', 'command torque', 'tau_b', legend_vec_rpy)
subplot(2,2,2)
plot_ts(u_T, 'time [sec]', 'command thrust', 'u_T', 'u_T')
subplot(2,2,3)
plot_ts(read_speed,'time [sec]','motor speed [rpm]', 'Read Motor Speed', {'m0','m1','m2','m3'});
subplot(2,2,4)
plot_ts(comm_speed,'time [sec]','motor speed [rpm]', 'Commanded Motor Speed', {'m0','m1','m2','m3'});


figure('Renderer', 'painters', 'Position', [1 73 1440 724])
subplot(2,2,1)
plot_ts(F,'time [sec]','F_tot[N]',"APF Total Force",legend_vec_xyz)
subplot(2,2,2)
plot_ts(F_rep,'time [sec]','F_tot[N]',"APF Repulsive Force",legend_vec_xyz)
subplot(2,2,3)
% plot_ts(dist3d,'time [sec]','distance[m]',"APF Obstacle distance",legend_vec_xyz)
% subplot(2,2,4)
% plot_ts(psi_des,'time [sec]','distance[m]',"APF Obstacle distance",legend_vec_xyz)
 

%% PLOT PATH 3D
% figure('Renderer', 'painters', 'Position', [1 73 1440 724])
% plot3(pos.Data(:,1),pos.Data(:,2),pos.Data(:,3))
% grid on
% axis equal
% view([-43.396800702791758 20.232570274887394])

%% PLOT PDF
legend_vec_xyz = {'$$x$$', '$$y$$', '$$z$$'};
legend_vec_rpy = {'$$roll$$', '$$pitch$$', '$$yaw$$'};
legend_vec_quat = {'$$x$$', '$$y$$', '$$z$$', '$$w$$'};
legend_vec_motor =  {'$$\omega_1$$','$$\omega_2$$','$$\omega_3$$','$$\omega_4$$'};

pdf_ts(q,       '$$time [sec]$$','$$position\ [m]$$',"Desired Position",legend_vec_xyz,'pos_d.pdf');
pdf_ts(dot_q,   '$$time [sec]$$','$$velocity\ [m/s]$$',"Desried Velocity",legend_vec_xyz,'vel_d.pdf');
pdf_ts(ddot_q,  '$$time [sec]$$','$$acceleration\ [m/s^2]$$',"Desried Acceleration",legend_vec_xyz,'acc_d.pdf');

pdf_ts(pos,     '$$time [sec]$$','$$position\ [m]$$',"Position",legend_vec_xyz,'pos.pdf');
pdf_ts(ori,     '$$time [sec]$$','$$quaternion$$',"Orientation",legend_vec_quat,'ori.pdf');
pdf_ts(vel,     '$$time [sec]$$','$$velocity\ [m/s]$$',"Linear Velocity",legend_vec_xyz,'lin_vel.pdf');
pdf_ts(ang_vel, '$$time [sec]$$','$$quaternion$$',"Angular Velocity",legend_vec_quat,'ang_vel.pdf');

pdf_ts(err_p,   '$$time [sec]$$','$$error\ pos\ [m]$$',"Error P",legend_vec_xyz,'err_p.pdf');
pdf_ts(err_v,   '$$time [sec]$$','$$error\ vel\ [m/s]$$',"Error V",legend_vec_xyz,'err_v.pdf');
pdf_ts(err_R,   '$$time [sec]$$','$$error\ rot\ [rad]$$',"Error R",legend_vec_xyz,'err_r.pdf');
pdf_ts(err_W,   '$$time [sec]$$','$$error\ ang\ vel\ [rad/s]$$',"Error W",legend_vec_xyz,'err_w.pdf');

pdf_ts(tau_b,   '$$time [sec]$$', '$$command\ torque$$', 'Command Torque $$\tau_{b}$$', legend_vec_rpy,'tau_b.pdf')
pdf_ts(u_T,     '$$time [sec]$$', '$$command\ thrust$$', 'Commanded Thrust $$u_{T}$$', '$$u_{T}$$','u_T.pdf')
pdf_ts(read_speed,'$$time [sec]$$','$$motor\ speed\ [rpm]$$', 'Read Motor Speed', legend_vec_motor,'read_speed.pdf');
pdf_ts(comm_speed,'$$time [sec]$$','$$motor\ speed\ [rpm]$$', 'Commanded Motor Speed', legend_vec_motor,'comm_speed.pdf');

pdf_ts(F,       '$$time [sec]$$','$$F_{tot}$$',"APF Total Force",legend_vec_xyz,'f_tot.pdf')
pdf_ts(F_rep,   '$$time [sec]$$','$$F_{rep}$$',"APF Repulsive Force",legend_vec_xyz,'f_rep.pdf')


