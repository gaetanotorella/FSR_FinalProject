close all
clear all
clc

bagfile = 'geom_controller.bag';
bag = rosbag(bagfile);
disp(bag.AvailableTopics);

bag_odometry = select(bag, 'Topic','/ardrone/odometry_sensor1/odometry');
bag_rpy_thrust = select(bag, 'Topic', '/ardrone/command/roll_pitch_yawrate_thrust1');
bag_f_e = select(bag, 'Topic','/ardrone/momentum_estimator/f_e');
bag_tau_e =  select(bag, 'Topic','/ardrone/momentum_estimator/tau_e');
bag_err_p = select(bag, 'Topic', '/ardrone/geom_control/err_p');

%% Leggi tutti i messaggi dai topic selezionati

pos = timeseries(bag_odometry,'Pose.Pose.Position.X','Pose.Pose.Position.Y','Pose.Pose.Position.Z');
ori = timeseries(bag_odometry,'Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z','Pose.Pose.Orientation.W');
vel = timeseries(bag_odometry,'Twist.Twist.Linear.X','Twist.Twist.Linear.Y','Twist.Twist.Linear.Z');
ang_vel = timeseries(bag_odometry,'Twist.Twist.Angular.X','Twist.Twist.Angular.Y','Twist.Twist.Angular.Z');                                                                                                                                                                                             
ros_f_e = timeseries(bag_f_e,'X','Y','Z');
ros_tau_e = timeseries(bag_tau_e,'X','Y','Z');
err_p = timeseries(bag_err_p,'X','Y','Z');


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


%% Estimator

T = ang_vel.time;

eta_b = quat2eul(ori.Data); %attitude.signals.values;
p_b_dot = vel.Data;
eta_b_dot = ang_vel.Data;
u_T = u_T.Data;
tau_values = tau_b.Data;

g = -9.81;
m = 1.5;
%m = 1.6288;
m = 1.6335;


Ib = diag([0.0347563 0.0458929 0.0977]);

C_0 = 8;
r = 1;
Ts = 1e-2;
q = zeros(6, length(T));
ext_wrench = zeros(6, length(T));
gamma = zeros(6, length(T), r);
K = compute_K_coefficients(C_0, r);

% Run the estimator
[ext_wrench, gamma, q] = run_estimator(T, m, g, Ts, r, K, q, ext_wrench, gamma, Ib, eta_b, p_b_dot, eta_b_dot, u_T, tau_values);


%% Plot results
plot_results(T, ext_wrench);

legend_vec_xyz = {'$$x$$', '$$y$$', '$$z$$'};
pdf_ts(ros_f_e,'$$time [sec]$$','$$f_{e}$$',"External Force",legend_vec_xyz,'f_e.pdf')
pdf_ts(ros_tau_e,'$$time [sec]$$','$$\tau_{e}$$',"External Torque",legend_vec_xyz,'tau_e.pdf')
pdf_ts(err_p,   '$$time [sec]$$','$$error\ pos\ [m]$$',"Error P",legend_vec_xyz,'err_p.pdf');
pdf_ts(thrusts,     '$$time [sec]$$', '$$command\ thrust$$', 'Commanded Thrust $$u_{T}$$', '$$u_{T}$$','u_T.pdf')
pdf_ts(pos,     '$$time [sec]$$','$$position\ [m]$$',"Position",legend_vec_xyz,'pos.pdf');

%% Compute real mass
m_real = compute_real_mass(m, g, ext_wrench);
display(m_real) % using the z-disturbance


%% UTILITY FUNCTIONS
function K = compute_K_coefficients(c_0, r)
    [num, den] = butter(r, c_0^(1/r), 'low', 's');
    G = tf(num, den);
    G = zpk(G);

    c = den(2:end); 
    K = zeros(r, 1);
    temp = 1;

    for i = 1:r
        K(i) = c(i) / temp;
        temp = temp * K(i);
    end

    K = flip(K);
end


function [ext_wrench, gamma, q] = run_estimator(T, m, g, Ts, r, K, q, ext_wrench, gamma, Ib, eta_b, p_b_dot, eta_b_dot, u_T, tau_values)
    for k = 1:length(T) - 1
        q_k_plus_1 = [m * eye(3) zeros(3, 3); zeros(3, 3) M_fun(eta_b(k + 1, :), Ib)] ...
            * [p_b_dot(k + 1, :)'; eta_b_dot(k + 1, :)'];

        C = C_fun(eta_b(k, :), eta_b_dot(k, :), Ib);
        Q = Q_fun(eta_b(k, :));
        Rb = R_fun(eta_b(k, :));

        gamma(:, k + 1, 1) = gamma(:, k, 1) + K(1) * ((q_k_plus_1 - q(:, k)) ...
            - Ts * [m * g * [0; 0; 1] - u_T(k) * Rb * [0; 0; 1]; ...
            C' * eta_b_dot(k, :)' + Q' * tau_values(k, :)'] ...
            - Ts * ext_wrench(:, k));

        gamma = update_gamma(gamma, k, r, Ts, K, ext_wrench);
        ext_wrench(:, k + 1) = gamma(:, k + 1, r);
        q(:, k + 1) = q_k_plus_1;
    end
end


function gamma = update_gamma(gamma, k, r, Ts, K, ext_wrench)
    if r >= 2
        for i = 2:r
            gamma(:, k + 1, i) = gamma(:, k, i) + Ts * K(i) * (-ext_wrench(:, k) + gamma(:, k, i - 1));
        end
    end
end


function plot_results(T, ext_wrench)
    f_ext_x = 0.5;
    f_ext_y = 0.5;
    tau_ext_yaw = 0.2;
    singleplot(T,ext_wrench,'time[sec]','external wrench estimated',{'$$f_{ex}$$','$$f_{ey}$$','$$f_{ez}$$','$$\tau_{ex}$$','$$\tau_{ey}$$','$$\tau_{ez}$$'},'wrench.pdf');

end


function m_real = compute_real_mass(m, g, ext_wrench)
    m_tilde = ext_wrench(3, end) / g;
    m_real = m_tilde + m; % computation using the z-disturbance
end
