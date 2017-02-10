
a=1.62; %Distance from CoG to front axle [m]
b=1.08; %Distance from CoG to rear axle [m]
m=1380; %mass of vehicle [kg]
l=2.70; %Wheelbase [m]

I=2.6611*10^3; %Yaw Moment of Inertia [kg*m^2]
c_f=77000; %Front wheel cornering stiffness [N/rad]
c_r=130000; %Rear wheel cornering stiffness [N/rad]


v_x=20; %Vehicle speed [m/s]
a_x=0; %Vehicle acceleration [m/s^2]


t = (0:0.1:10)';

A=[(-(a^2)*c_f-(b^2)*c_r)/(I*v_x) (-(a)*c_f+(b)*c_r)/I 0 0;...
(-(a)*c_f+(b)*c_r-m*(v_x)^2)/(m*(v_x)^2),(-c_f-c_r)/(m*(v_x)) 0 0;...
    1,0,0,0;
    0,v_x,v_x,0];
B=[a*c_f/I,0;...
    c_f/(m*c_f),0;...
    0,-v_x;
    0,0];
C=[1,0,0,0;...
    (-(a)*c_f+(b)*c_r)/(m*v_x),(-c_f-c_r+m*a_x)/m,0,0;...
    0,0,1,0;...
    0,0,0,1];
D=[0,0;...
    c_f/(m),0;...
    0,0;
    0,0];

sys=ss(A,B,C,D);

%Stability check
eigs=eig(A);

%Controllability Check
% (rank(ctrb(sys))==4)
%Observability check
% (rank(obsv(sys))==4)

%% Pole positioning attempt

k=1;
P=[-k+1i,-k-1i,-500*k+1i,-500*k-1i];
 K=place(A,B,P);
sys_1=ss(A-B*K,B,C-D*K,D);

k=0.75;
P=[-k+1i,-k-1i,-500*k+1i,-500*k-1i];
 K=place(A,B,P);
sys_2=ss(A-B*K,B,C-D*K,D);

k=0.5;
P=[-k+1i,-k-1i,-500*k+1i,-500*k-1i];
 K=place(A,B,P);
sys_3=ss(A-B*K,B,C-D*K,D);

figure(1)
 step(sys_1,'b',sys_2,'r',sys_3,'g',t)
 legend('1','2','3')
grid

% P=[-5+1i,-5-1i,-10+1i,-10-1i];
%  K=place(A,B,P);
% sys_1=ss(A-B*K,B,C-D*K,D);
% 
% P=[-2+1i,-2-1i,-4+1i,-4-1i];
%  K=place(A,B,P);
% sys_2=ss(A-B*K,B,C-D*K,D);
% 
% P=[-1+1i,-1-1i,-2+1i,-2-1i];
%  K=place(A,B,P);
% sys_3=ss(A-B*K,B,C-D*K,D);
% 
% figure(1)
%  step(sys_1,'b',sys_2,'r',sys_3,'g',t)
%  legend('1','2','3')
% grid

figure(2)
pzmap(sys,sys_cl)

figure(3)
iopzplot(sys_1)

figure(4)
sigma(sys)
grid
grid