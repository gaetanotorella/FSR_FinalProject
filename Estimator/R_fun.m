function Rb = R_fun(eta_b)
Rb=[cos(eta_b(2))*cos(eta_b(3)) sin(eta_b(1))*sin(eta_b(2))*cos(eta_b(3))-cos(eta_b(1))*sin(eta_b(3)) cos(eta_b(1))*sin(eta_b(2))*cos(eta_b(3))+sin(eta_b(1))*sin(eta_b(3));
    cos(eta_b(2))*sin(eta_b(3)) sin(eta_b(1))*sin(eta_b(2))*sin(eta_b(3))+cos(eta_b(1))*cos(eta_b(3)) cos(eta_b(1))*sin(eta_b(2))*sin(eta_b(3))-sin(eta_b(1))*cos(eta_b(3));
-sin(eta_b(2)) sin(eta_b(1))*cos(eta_b(2)) cos(eta_b(1))*cos(eta_b(2))];
end %matrice di rotazione data dagi anfoli di Eulero slide 38