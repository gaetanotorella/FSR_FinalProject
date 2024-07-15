function M=M_fun(eta_b,Ib)

Q=Q_fun(eta_b);

M=Q'*Ib*Q;


end