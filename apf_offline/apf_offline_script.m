clear all
close all
clc

g = 9.81;
e_3 = [0 0 1]';
uT_0 = 14.92;      
Ts = 0.001; 
t_end = 40;

%% UAV PARAMETERS

mass = 1.52;
Ib = diag([0.0347563 0.0458929 0.0977]);
kp = diag([6 6 6]);
kv = diag([4.7 4.7 4.7]);
kr = diag([2 2.3 0.15]);
kw = diag([0.4 0.52 0.18]);

%% ARTIFICIAL POTENTIAL PARAM

d0 = 2.5;           % coeff influenza
k_att = 0.8;
k_rep = 0.03;
q_0 = [0 0 0];
q_goal = [10 10 -10];

%% SCENARIO
obs_1 = [4.5 5 0.3 0 10];
obs_2 = [2 2 0.2 0 3];

q_obstacles = [obs_points_gen(obs_1); obs_points_gen(obs_2)];

scenario_generator;


%% SIM
close all
open_system("apf_offline.slx")
out = sim("apf_offline.slx",'StopTime','40');

legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(time,out.F,'time[sec]','$$Force$$',"Total Force",legend_vec,'f_tot.pdf');
legend_vec = {'$$\tau_{x}$$','$$\tau_{y}$$','$$\tau_{z}$$'};
singleplot(time,out.tau_b.Data,'time[sec]','toruqe\ [Nm]',"Commanded Torque",legend_vec,'commTau.pdf');


%% PLOT PDF
close all

time = linspace(0,t_end,length(out.err_p.Data));
legend_vec = {'$$err_{x}$$','$$err_{y}$$','$$err_{z}$$'};
singleplot(time,out.err_p.Data,'time[sec]','position[m]',"Position error",legend_vec,'errPos.pdf');

legend_vec = {'$$err_{\dot x}$$','$$err_{\dot y}$$','$$err_{\dot z}$$'};
singleplot(time,out.dot_err_p.Data,'time[sec]','velocity[m/s]',"Velocity error",legend_vec,'errVel.pdf');

legend_vec = {'$$err_{R1}$$','$$err_{R2}$$','$$err_{R3}$$'};
singleplot(time,out.err_R.Data,'time[sec]',' ',"Orientation error",legend_vec,'errOrient.pdf');

legend_vec = {'$$err_{W1}$$','$$err_{W2}$$','$$err_{W3}$$'};
singleplot(time,out.err_W.Data,'time[sec]',' ',"Orientation Velocity error",legend_vec,'errOrientVel.pdf');
 
legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(time,out.pos,'time[sec]','$$position\ [m]$$',"Position",legend_vec,'pos.pdf');

legend_vec = {'$$w$$', '$$x$$', '$$y$$','$$z$$'};
singleplot(time,rotm2quat(out.rot.Data),'time[sec]','$$rotation\ [rad]$$',"Rotation",legend_vec,'rot.pdf');

legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(time,out.F,'time[sec]','$$Force$$',"Total Force",legend_vec,'f_tot.pdf');
singleplot(time,out.F_rep,'time[sec]','$$Force$$',"Repulsive Force",legend_vec,'f_rep.pdf');
singleplot(time,out.F_a,'time[sec]','$$Force$$',"Attractive Force",legend_vec,'f_a.pdf');

legend_vec = {'$$\tau_{x}$$','$$\tau_{y}$$','$$\tau_{z}$$'};
singleplot(time,out.tau_b.Data,'time[sec]','toruqe\ [Nm]',"Commanded Torque",legend_vec,'commTau.pdf');

legend_vec = {'$$u_T$$'};
singleplot(time,out.uT.Data,'time[sec]','$$u_{T}\ [N]$$',"Commanded Thrust",legend_vec,'commThrust.pdf');

legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(time,out.pos_d(1:3,:),'time[sec]','$$position\ [m]$$',"Desired Position",legend_vec,'pos_d.pdf');

legend_vec = {'$$\dot{x}$$','$$\dot{y}$$','$$\dot{z}$$'};
singleplot(time,out.vel_d,'time[sec]','velocity\ [m/s]',"Desired Velocity",legend_vec,'vel_d.pdf');

legend_vec = {'$$\ddot{x}$$','$$\ddot{y}$$','$$\ddot{z}$$'};
singleplot(time,out.acc_d(1:3,:),'time[sec]','$$acceleration\ [m/s^2]$$',"Desired Acceleration",legend_vec,'acc_d.pdf');

