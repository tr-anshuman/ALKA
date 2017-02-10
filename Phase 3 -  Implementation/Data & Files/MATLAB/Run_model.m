close all;
clear all;
clc;

%% vehicle paramaters
a=1.62;             % Distance from CoG to front axle [m]
b=1.08;             % Distance from CoG to rear axle [m]
m=1380;             % mass of vehicle [kg]
l=a+b;              % Wheelbase [m]
I_z=2.6611*10^3;    % Yaw Moment of Inertia [kg*m^2]
c_f=77000;          % Front wheel cornering stiffness [N/rad]
c_r=130000;         % Rear wheel cornering stiffness [N/rad]

dt=0.001;            % Sample time
v_x=20;              % Longitudinal velocity in m/s; %[m/s^2]
MPH=v_x*3.6*0.621371;% transform km/h to m/s
L=0.4*MPH+0.2;       % Calculate longitudinal look ahead distance


%% ss matrices
A=[(-((a^2)*c_f+(b^2)*c_r))/(I_z*v_x) (-(a)*c_f+(b)*c_r)/(I_z*v_x) 0 0;...
(-(a)*c_f+(b)*c_r-m*(v_x)^2)/(m*(v_x)) (-c_f-c_r)/(m*(v_x)) 0 0;...
    -1,0,0,0;
    -L,-1,v_x,0];
B=[a*c_f/I_z,0;...
    c_f/(m),0;...
    0,v_x;
    0,0];
C=[1,0,0,0;...
    0,1,0,0;...
    0,0,1,0;...
    0,0,0,1;...
    0,1/v_x,0,0;...
    0,1,v_x,0];

D=[0 0; 0 0; 0 0; 0 0; 0 0; 0 0];

%% check the ss matrices
% bicycle=ss(A,B,C,D);
% eig(A);
% 
% rank(ctrb(bicycle));
% rank(obsv(bicycle));

%% select either one of these simulations! 
sim('BicycleMDL_sine.slx')              % with sine input
% sim('BicycleMDL_step.slx')              % with step

%% plots
figure(1)
plot(v_x*e_1.time,e_1.data,'b')
hold on;
plot(v_x*e_1.time,zeros(1,length(e_1.data)),'r--')
plot(v_x*e_1.time,1.5*ones(1,length(e_1.data)),'r')
plot(v_x*e_1.time,(-1.5)*ones(1,length(e_1.data)),'r')
xlabel('x [m]');
ylabel('y [m]');
legend('trajectory','straight line reference')

figure(2)
plot(v_x*e_1.time,beta.data*180/pi,'b')
hold on;
xlabel('x [m]');
ylabel('Beta [deg]');

figure(3)
plot(v_x*e_1.time,vy.data,'b')
hold on;
xlabel('x [m]');
ylabel('Lateral velocity V_y [m/s]');

figure(4)
plot(v_x*e_1.time,psi_veh.data*180/pi,'b')
hold on;
xlabel('x [m]');
ylabel('Vehicle yawrate [deg/s]');