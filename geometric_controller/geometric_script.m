clear all
close all
clc

g = 9.81;
e_3 = [0 0 1]';
uT_0 = 11.772;      % thrust at the steady-state 
%uT_0 = 12;

mass = 1.2;
Ib = diag([1.2416 1.2416 2*1.2416]);

Ts=0.001; % SAMPLING TIME

%% Gains

% Kp = diag([100 100 150]);
% Kv = diag([30 30 30]);
% Kr = diag([50 50 50]);
% Kw = diag([25 25 25]);

Kp = diag([60 60 75]);
Kv = diag([1 1 2]);
Kr = diag([50 50 70]);
Kw = diag([5 5 5]);

%% SIM
waypoints_generator;

out = sim("geometric_control_template.slx",'StopTime','30');

%% DATA F
% f_1 = out.F.Data(1,:);
% f_2 = out.F.Data(2,:);
% f_3 = out.F.Data(3,:);
% figure()
% plot3(f_1, f_2, f_3)
% grid on

%% Plots

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
title("Errors")
subplot(2,2,1); 
legend_vec = {'$$err_{x}$$','$$err_{y}$$','$$err_{z}$$'};
singleplot(out.tout,out.err_p.Data,'time[sec]','position[m]',"Position error",legend_vec,'errPos.pdf');

subplot(2,2,2); 
legend_vec = {'$$err_{\dot x}$$','$$err_{\dot y}$$','$$err_{\dot z}$$'};
singleplot(out.tout,out.dot_err_p.Data,'time[sec]','velocity[m/s]',"Velocity error",legend_vec,'errVel.pdf');

subplot(2,2,3); 
legend_vec = {'$$err_{R1}$$','$$err_{R2}$$','$$err_{R3}$$'};
singleplot(out.tout,out.err_R.Data,'time[sec]',' ',"Orientation error",legend_vec,'errOrient.pdf');

subplot(2,2,4); 
legend_vec = {'$$err_{W1}$$','$$err_{W2}$$','$$err_{W3}$$'};
singleplot(out.tout,out.err_W.Data,'time[sec]',' ',"Orientation Velocity error",legend_vec,'errOrientVel.pdf');

figure('Renderer', 'painters', 'Position', [1 73 1440 724])
title("Data")
subplot(2,2,1); 
legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(out.tout,out.pos.Data,'time[sec]','$$position\ [m]$$',"Position",legend_vec,'commThrust.pdf');

subplot(2,2,2); 
%legend_vec = {'$$x_b$$', '$$y_b$$', '$$z_b$$'};
%singleplot(out.tout,out.rot.Data,'time[sec]','$$rotation\ [rad]$$',"Rotation",legend_vec,'commThrust.pdf');

subplot(2,2,3); 
legend_vec = {'$$\tau_{x}$$','$$\tau_{y}$$','$$\tau_{z}$$'};
singleplot(out.tout,out.tau_b.Data,'time[sec]','toruqe\ [Nm]',"Commanded Torque",legend_vec,'commTorque.pdf');

subplot(2,2,4); 
legend_vec = {'$$u_T$$'};
singleplot(out.tout,out.uT.Data,'time[sec]','$$u_{T}\ [N]$$',"Commanded Thrust",legend_vec,'commThrust.pdf');

