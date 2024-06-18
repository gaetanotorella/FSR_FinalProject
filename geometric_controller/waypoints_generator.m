dt = 0.1;
t_end = 30;
Ts=0.001; % SAMPLING TIME

q_0 = [1 1 -4]';
q_f = [4 4 -6]';
q_obs_1 = [4.5; 5; -5];
q_obs_2 = [5; 4.5; -5];
q_obs_3 = [5; 4.5; -4];
q_obs_4 = [7; 7; -7];
q_obstacles = [q_obs_1 q_obs_2 q_obs_3 q_obs_4];

d0 = 2.5;           % coeff influenza
k_att = 0.4;
k_rep = 0.8;
k_v = 0.8;
k_a = 1;
gamma = 3;

q = q_0;
traj = [q];
e_pos = q_f - q;

for i = 1:dt:30

    e_pos = q_f - q;

    if(norm(e_pos)<1)
        F_a = k_att * e_pos;
    else
        F_a = k_att * e_pos/norm(e_pos);
    end

    % Inizializza la forza repulsiva
    F_rep = zeros(3, 1);

    % Calcola la forza repulsiva per ciascun ostacolo
    for i = 1:size(q_obstacles, 2)
        q_obs = q_obstacles(:, i);
        dist = norm(q - q_obs);

        if dist <= d0
            eta_q = dist;
            eta_q0 = d0;
            grad_eta_q = (q - q_obs) / dist;

            F_rep = F_rep + (k_rep/(eta_q^2))*(1/eta_q - 1/eta_q0)^(gamma-1) * grad_eta_q;
        end
    end

    F = F_a + F_rep;

    % spostiamo q

    q = q + F * dt;
    traj = [traj q];
    if q == q_f
        i = 30
    end
end

newtraj = [traj; zeros(1,size(traj,2))];
traj = newtraj;
%% PLOT

figure()
plot3(traj(1,:),traj(2,:),-traj(3,:))
axis equal
view([-43.396800702791758 20.232570274887394])

hold on
grid on
plot3(q_0(1),q_0(2),-q_0(3),"o",'Color',"b",'MarkerSize',10)
plot3(q_f(1),q_f(2),-q_f(3),"o",'Color',"g",'MarkerSize',10)

for i = 1:size(q_obstacles, 2)
plot3(q_obstacles(1,i),q_obstacles(2,i),-q_obstacles(3,i),"*",'Color',"r",'MarkerSize',10)
end


%% TRAJECTORY

t_interval = linspace(0,t_end,size(traj,2));

% Definisci le velocità iniziali e finali per ciascun waypoint
initialVelocity = ones(2,size(traj,2));
finalVelocity = ones(2,size(traj,2));
bound_Velocity = [initialVelocity; finalVelocity];
bound_Velocity(:,1) = 0;
bound_Velocity(:,end) = 0;

acc = 10*ones(1,size(traj,2));
bound_Acceleration = [acc; acc; acc; acc];
bound_Acceleration(:,1) = 0;
bound_Acceleration(:,end) = 0;

t = 0:Ts:t_end;
% [p_d, dot_p_d, ddot_p_d, pp] = quinticpolytraj(traj, t_interval, t, 'VelocityBoundaryCondition', bound_Velocity,'AccelerationBoundaryCondition',bound_Acceleration);
[p_d, dot_p_d, ddot_p_d, pp, tPoints,tSamples] = minjerkpolytraj(traj, t_interval, 30001);  %,'AccelerationBoundaryCondition',bound_Acceleration);

%% data for Simulink
pos_0 = q_0';
lin_vel_0 = [0 0 0];
w_bb_0 = [0 0 0];
csi_d = p_d(1:3,:); 
dot_csi_d = dot_p_d(1:3,:); 
ddot_csi_d = ddot_p_d(1:3,:);
psi_d = p_d(4,:); 
dot_psi_d = dot_p_d(4,:); 
ddot_psi_d = ddot_p_d(4,:);

%% PLOTS
figure('Renderer', 'painters', 'Position', [1 73 1440 724])
title("PLANNER")
subplot(2,2,1); 
legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(t,csi_d,'time[sec]','$$position\ [m]$$',"Desired Position",legend_vec,'commThrust.pdf');

subplot(2,2,2); 
legend_vec = {'$$\dot{x}$$','$$\dot{y}$$','$$\dot{z}$$'};
singleplot(t,dot_csi_d,'time[sec]','velocity\ [m/s]',"Desired Velocity",legend_vec,'commTorque.pdf');

subplot(2,2,3); 
legend_vec = {'$$\ddot{x}$$','$$\ddot{y}$$','$$\ddot{z}$$'};
singleplot(t,ddot_csi_d,'time[sec]','$$acceleration\ [m/s^2]$$',"Desired Acceleration",legend_vec,'commThrust.pdf');

