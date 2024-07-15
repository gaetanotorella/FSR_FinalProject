dt = 0.1;
t_end = 40;

q_0 = [0 0 0]';
q_f = [10 10 -10]';

% xcentro ycentro r zcentro h
obs_1 = [4.5 5 0.3 0 10];
obs_2 = [2 2 0.2 0 3];

q_obstacles = [obs_points_gen(obs_1) obs_points_gen(obs_2)];

d0 = 2.5;           % coeff influenza
k_att = 0.8;
k_rep = 0.8;

gamma = 3;

q = q_0;
traj = [q];
e_pos = q_f - q;

plot_F = []; 
plot_F_rep = [];
plot_F_a = [];

for k = 0:dt:t_end

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

    F_tot = F_a + F_rep;

    alpha = calculate_alpha(k);
    F = exp_mov_average(F_tot,alpha);

    plot_F_a = [plot_F_a F_a];
    plot_F_rep = [plot_F_rep F_rep];
    plot_F = [plot_F F];

    q = q + F * dt;
    traj = [traj q];

end

newtraj = [traj; zeros(1,size(traj,2))];
traj = newtraj;


%% TRAJECTORY

t_interval = 0:dt:t_end+dt;

t = 0:Ts:t_end;

[p_d, dot_p_d, ddot_p_d, pp, tPoints,tSamples] = minjerkpolytraj(traj, t_interval, length(t));


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


%% PLOT 3D

fig_traj = figure('Renderer', 'painters', 'Position', [1,72,727,725]);
removeToolbarExplorationButtons(fig_traj)
plot3(traj(1,:),traj(2,:),-traj(3,:))
xlabel("East [m]")
ylabel("North [m]")
zlabel("Up [m]")
axis equal
view([-18.085159810409245 19.491063829787073])
hold on
grid on
plot3(q_0(1),q_0(2),-q_0(3),"o",'Color',"b",'MarkerSize',10)
plot3(q_f(1),q_f(2),-q_f(3),"o",'Color',"g",'MarkerSize',10)

for i = 1:size(q_obstacles, 2)
plot3(q_obstacles(1,i),q_obstacles(2,i),-q_obstacles(3,i),"*",'Color',"r",'MarkerSize',10)
end
axis on
exportgraphics(fig_traj, "traj3d_d.pdf");

%% gif
for n = 0:1:90
      view([n 19.491063829787073])
      exportgraphics(gcf,'testAnimated.gif','Append',true);
end

set(gca, 'CameraPosition', [-20  -72   33]);


%% PDF

legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(t,csi_d,'time[sec]','$$position\ [m]$$',"Desired Position",legend_vec,'pos_d.pdf');

legend_vec = {'$$\dot{x}$$','$$\dot{y}$$','$$\dot{z}$$'};
singleplot(t,dot_csi_d,'time[sec]','velocity\ [m/s]',"Desired Velocity",legend_vec,'vel_d.pdf');

legend_vec = {'$$\ddot{x}$$','$$\ddot{y}$$','$$\ddot{z}$$'};
singleplot(t,ddot_csi_d,'time[sec]','$$acceleration\ [m/s^2]$$',"Desired Acceleration",legend_vec,'acc_d.pdf');

legend_vec = {'$$\psi_d$$','$$\dot{\psi}_d$$','$$\ddot{\psi}_d$$'};
singleplot(t,[psi_d; dot_psi_d;ddot_psi_d],'time[sec]','$$rotation\ [m/s^2]$$',"Desired Rotation",legend_vec,'psi_d.pdf');

%% PLOT F

f_t = linspace(0,t_end,length(plot_F));
legend_vec = {'$$x$$', '$$y$$', '$$z$$'};
singleplot(f_t,plot_F,'time[sec]','$$Force$$',"Total Force",legend_vec,'f_tot.pdf');
singleplot(f_t,plot_F_rep,'time[sec]','$$Force$$',"Repulsive Force",legend_vec,'f_rep.pdf');
singleplot(f_t,plot_F_a,'time[sec]','$$Force$$',"Attractive Force",legend_vec,'f_a.pdf');

