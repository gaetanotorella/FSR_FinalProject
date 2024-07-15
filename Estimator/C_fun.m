function [C , Q, Q_dot]=C_fun(eta_b,eta_b_dot,Ib)

Q=Q_fun(eta_b); 
S=skew(Q*eta_b_dot');
Q_dot=Q_dot_fun(eta_b,eta_b_dot);
C=Q'*S*Ib*Q+Q'*Ib*Q_dot; %modello dinamico RPY UAV


end