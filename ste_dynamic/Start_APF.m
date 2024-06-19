clear all
close all
clc

%% Data
% 
% kp=100;
% kv=7;
% 
% kr=200;
% kw=5;

%% BUONI
% Gain Art pot. method
k_att=0.5;
k_rep=0.5;
%Gain ref generator
k_v=0.8;
k_a=1;

kp=diag([100 160 175]);
kv=diag([10 10 20]);
kr=diag([150 150 170]);
kw=diag([50 50 50]);

%% Sim
open_system('dynamic_APF.slx');
out = sim("dynamic_APF.slx",'StopTime','60');

%% Plots
legend_vec = {'$$err_{x}$$','$$err_{y}$$','$$err_{z}$$'};
singleplot(out.tout,out.err_p.Data,'time[sec]','position[m]',"Position error",legend_vec,'errPos.pdf');

legend_vec = {'$$err_{\dot x}$$','$$err_{\dot y}$$','$$err_{\dot z}$$'};
singleplot(out.tout,out.dot_err_p.Data,'time[sec]','velocity[m/s]',"Velocity error",legend_vec,'errVel.pdf');

legend_vec = {'$$err_{R1}$$','$$err_{R2}$$','$$err_{R3}$$'};
singleplot(out.tout,out.err_R.Data,'time[sec]',' ',"Orientation error",legend_vec,'errOrient.pdf');

legend_vec = {'$$err_{W1}$$','$$err_{W2}$$','$$err_{W3}$$'};
singleplot(out.tout,out.err_w.Data,'time[sec]',' ',"Orientation Velocity error",legend_vec,'errOrientVel.pdf');

legend_vec = {'$$u_T$$'};
singleplot(out.tout,out.uT.Data,'time[sec]','$$u_{T}\ [N]$$',"Commanded Thrust",legend_vec,'commThrust.pdf');

legend_vec = {'$$\tau_{x}$$','$$\tau_{y}$$','$$\tau_{z}$$'};
singleplot(out.tout,out.tau_b.Data,'time[sec]','toruqe\ [Nm]',"Commanded Torque",legend_vec,'commTorque.pdf');

% figure()
% title("Position error")
% subplot(2,1,1); 
% plot(out.err_p,'LineWidth', 1.5);
% title('p_err');
% xlabel('t[s]');
% ylabel('[m]');
% grid on
% legend('err x','err y','err z')
% 
% subplot(2,1,2); 
% plot(out.dot_err_p,'LineWidth', 1.5);
% title('pDot_err');
% xlabel('t[s]');
% ylabel('[m/s]');
% grid on
% legend('err x_dot','err y_dot','err z_dot')
% 
% 
% figure()
% title("Orientation error")
% % Subplot 1
% subplot(2,1,1); 
% plot(out.err_R,'LineWidth', 1.5);
% title('R error');
% xlabel('t[s]');
% ylabel('');
% grid on
% legend('err R1','err R2','err R3')
% 
% subplot(2,1,2); 
% plot(out.err_w,'LineWidth', 1.5);
% title('W error');
% xlabel('t[s]');
% ylabel('');
% grid on
% legend('err W1','err W2','err W3')
% 
% figure()
% title("Command")
% subplot(2,1,1); 
% plot(out.uT,'LineWidth', 1.5);
% title('u_t');
% xlabel('t[s]');
% ylabel('');
% grid on
% 
% subplot(2,1,2); 
% plot(out.tau_b,'LineWidth', 1.5);
% title('tau_b');
% xlabel('t[s]');
% ylabel('');
% grid on
% legend('tau x','tau y','tau z')
% 
% figure()
% title("Error norm pos")
% subplot(2,1,1); 
% plot(out.err_p1,'LineWidth', 1.5);
% %title('u_t');
% xlabel('t[s]');
% ylabel('');
% grid on
% 
% subplot(2,1,2); 
% plot(out.dot_err_p1,'LineWidth', 1.5);
% %title('tau_b');
% xlabel('t[s]');
% ylabel('');
% grid on
% %legend('tau x','tau y','tau z')
% 
% figure()
% title("Error norm Attitude")
% subplot(2,1,1); 
% plot(out.err_R1,'LineWidth', 1.5);
% %title('u_t');
% xlabel('t[s]');
% ylabel('');
% grid on
% 
% subplot(2,1,2); 
% plot(out.err_w1,'LineWidth', 1.5);
% %title('tau_b');
% %xlabel('t[s]');
% ylabel('');
% grid on
% %legend('tau x','tau y','tau z')