function Q = Q_fun(RPY)
Q=[1 0 -sin(RPY(2));
    0 cos(RPY(1)) cos(RPY(2))*sin(RPY(1))
    0 -sin(RPY(1)) cos(RPY(1))*cos(RPY(2))];
end