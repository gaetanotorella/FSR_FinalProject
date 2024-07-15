function Q_dot=Q_dot_fun(eta_b,eta_b_dot)

Q_dot=[0 0 -cos(eta_b(2))*eta_b_dot(2);
0 -sin(eta_b(1))*eta_b_dot(1) -sin(eta_b(2))*sin(eta_b(1))*eta_b_dot(2)+cos(eta_b(2))*cos(eta_b(1))*eta_b_dot(1);
0 -cos(eta_b(1))*eta_b_dot(1) -sin(eta_b(2))*cos(eta_b(1))*eta_b_dot(2)-cos(eta_b(2))*sin(eta_b(1))*eta_b_dot(1)];
end