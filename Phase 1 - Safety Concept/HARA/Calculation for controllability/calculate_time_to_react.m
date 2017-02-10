R_min=[120 240 450 650 1000];
w_lane=3.0;
v_x=[60 80 100 120 140]/3.6;
for i = 1:1:length(v_x)
x(i)=sqrt((R_min(i)+0.5*w_lane)^2 -R_min(i)^2);  % available distance to avoid exceeding lane
t(i)=x(i)/v_x(i);                                % available reaction time
end

t