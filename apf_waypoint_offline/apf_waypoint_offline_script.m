clear all
close all
clc

g = 9.81;
e_3 = [0 0 1]';
uT_0 = 14.92;      
Ts = 0.001; 

%% ROS params

mass = 1.52;
Ib = diag([0.0347563 0.0458929 0.0977]);
Kp = diag([6 6 6]);
Kv = diag([4.7 4.7 4.7]);
Kr = diag([2 2.3 0.15]);
Kw = diag([0.4 0.52 0.18]);

%% CONFIGURATION

waypoints_generator;
scenario_generator;


%% SIM

out = sim("apf_waypoint_offline.slx",'StopTime','40');


%% PLOT PDF
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
singleplot(time,out.pos.Data,'time[sec]','$$position\ [m]$$',"Position",legend_vec,'pos.pdf');

legend_vec = {'$$w$$', '$$x$$', '$$y$$','$$z$$'};
singleplot(time,rotm2quat(out.rot.Data),'time[sec]','$$rotation\ [rad]$$',"Rotation",legend_vec,'rot.pdf');

legend_vec = {'$$\tau_{x}$$','$$\tau_{y}$$','$$\tau_{z}$$'};
singleplot(time,out.tau_b.Data,'time[sec]','toruqe\ [Nm]',"Commanded Torque",legend_vec,'commTau.pdf');

legend_vec = {'$$u_T$$'};
singleplot(time,out.uT.Data,'time[sec]','$$u_{T}\ [N]$$',"Commanded Thrust",legend_vec,'commThrust.pdf');

