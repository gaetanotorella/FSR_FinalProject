function S=skew(w)

if(numel(w)~= 1)

S= [0 -w(3) w(2);
w(3) 0 -w(1);
-w(2) w(1) 0];

else

S= zeros(3);

end

end


