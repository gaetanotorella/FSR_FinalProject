%% INITIALIZATION

%% UAV PARAMETERS

mass = 1.2;
Ib = diag([1.2416 1.2416 2*1.2416]);

%% SAMPLING TIME
Ts=0.001;

%% PLANNER
t_iniz = 0;
ttot = 20; %duration
tdead = 10; %dead time to evaluate the steady-state
tot_time = ttot + tdead;
t1 = linspace(0,ttot,round(ttot/Ts));
t = linspace(0,ttot+tdead,round(ttot/Ts)+round(tdead/Ts));

% pos = [x y z ɸ]
p = zeros(4,length(t1)); dot_p = zeros(4,length(t1)); ddot_p = zeros(4,length(t1));
p_d = zeros(4,length(t)); dot_p_d = zeros(4,length(t)); ddot_p_d = zeros(4,length(t));

% Initial conditions
x0 = 0; xf=1; dot_x0= 0; dot_xf=0; ddot_x0=0; ddot_xf=0; dddot_x0 = 0; dddot_xf = 0;
y0 = 0; yf=1; dot_y0= 0; dot_yf=0; ddot_y0=0; ddot_yf=0; dddot_y0 = 0; dddot_yf = 0;
z0 = -1; zf=-4; dot_z0= 0; dot_zf=0; ddot_z0=0; ddot_zf=0; dddot_z0 = 0; dddot_zf = 0;
psi0 = 0;

% Final conditions

psif = 0; %deg2rad(20);
dot_psi0= 0; dot_psif=0; ddot_psi0=0; ddot_psif=0; dddot_psi0 = 0; dddot_psif = 0;
p0=[x0,y0,z0,psi0]; dot_p0=[dot_x0,dot_y0,dot_z0,dot_psi0]; ddot_p0=[ddot_x0,ddot_y0,ddot_z0,ddot_psi0]; dddot_p0=[dddot_x0,dddot_y0,dddot_z0,dddot_psi0];
pf=[xf,yf,zf,psif]; dot_pf=[dot_xf,dot_yf,dot_zf,dot_psif]; ddot_pf=[ddot_xf,ddot_yf,ddot_zf,ddot_psif]; dddot_pf=[dddot_xf,dddot_yf,dddot_zf,dddot_psif];

%5-th order polynomial
a0=zeros(1,4); a1=zeros(1,4); a2=zeros(1,4); a3=zeros(1,4); a4=zeros(1,4); a5=zeros(1,4); a6 = zeros(1,4); a7 = zeros(1,4);

for j=1:4
    A = [t_iniz^7, t_iniz^6, t_iniz^5, t_iniz^4, t_iniz^3, t_iniz^2, t_iniz, 1;
        ttot^7, ttot^6, ttot^5, ttot^4, ttot^3, ttot^2, ttot, 1;
        7*t_iniz^6, 6*t_iniz^5, 5*t_iniz^4, 4*t_iniz^3, 3*t_iniz^2, 2*t_iniz, 1, 0;
        7*ttot^6, 6*ttot^5, 5*ttot^4, 4*ttot^3, 3*ttot^2, 2*ttot, 1, 0;
        42*t_iniz^5, 30*t_iniz^4, 20*t_iniz^3, 12*t_iniz^2, 6*t_iniz, 2, 0, 0;
        42*ttot^5, 30*ttot^4, 20*ttot^3, 12*ttot^2, 6*ttot, 2, 0, 0;
        210*t_iniz^4, 120*t_iniz^3, 60*t_iniz^2, 24*t_iniz, 6, 0, 0, 0;
        210*ttot^4, 120*ttot^3, 60*ttot^2, 24*ttot, 6, 0, 0, 0];
    b = [p0(j) pf(j) dot_p0(j) dot_pf(j) ddot_p0(j) ddot_pf(j) dddot_p0(j) dddot_pf(j)]';
    a_temp = A\b;
    a7(j) = a_temp(1);
    a6(j) = a_temp(2);
    a5(j) = a_temp(3);
    a4(j) = a_temp(4);
    a3(j) = a_temp(5);
    a2(j) = a_temp(6);
    a1(j) = a_temp(7);
    a0(j) = a_temp(8);

    %trajectories
    p(j,:)=a7(j)*t1.^7 + a6(j)*t1.^6 + a5(j)*t1.^5 +a4(j)*t1.^4 +a3(j)*t1.^3 +a2(j)*t1.^2 +a1(j)*t1 +a0(j);
    dot_p(j,:) = 7*a7(j)*t1.^6 + 6*a6(j)*t1.^5 + 5*a5(j)*t1.^4 +4*a4(j)*t1.^3 +3*a3(j)*t1.^2 +2*a2(j)*t1 +a1(j);
    ddot_p(j,:) = 42*a7(j)*t1.^5 + 30*a6(j)*t1.^4 + 5*4*a5(j)*t1.^3 +4*3*a4(j)*t1.^2 +3*2*a3(j)*t1 +2*a2(j);
    %addition of the stead-state terms
    p_d(j,:)=[p(j,:) zeros(1,round(tdead/Ts))+pf(j)];
    dot_p_d(j,:)=[dot_p(j,:) zeros(1,round(tdead/Ts))+dot_pf(j)];
    ddot_p_d(j,:)=[ddot_p(j,:) zeros(1,round(tdead/Ts))+ddot_pf(j)];

end

%data for Simulink
pos_0 = [x0 y0 z0];
lin_vel_0 = [dot_x0 dot_y0 dot_z0];
w_bb_0 = [0 0 0];
csi_d=p_d(1:3,:); dot_csi_d=dot_p_d(1:3,:); ddot_csi_d=ddot_p_d(1:3,:);
psi_d=p_d(4,:); dot_psi_d=dot_p_d(4,:); ddot_psi_d=ddot_p_d(4,:);
