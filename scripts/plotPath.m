function [] = plotPath(fileName)
% PLOTPATH Plots a trajectory file from mpp::save_to_txt()
%

%  Time [s]     x_global [m]    y_global [m]       phi [rad]   vx_local [m]
%     1              2            3                   4              5
%
%  vy_local[m]   omega [rad/s]       PTG_index  PTG_traj_index     PTG_step
%       6           7                  8           9          10
%
D=load(fileName);

t=D(:,1);
x=D(:,2);
y=D(:,3);
phi=D(:,4);
vx=D(:,5);
vy=D(:,6);
w=D(:,7);
PTG_index=D(:,8);
PTG_trajIndex=D(:,9);

figure;
plot(x,y,'.');
grid minor;
axis equal;
xlabel('x');
ylabel('y');
title('Trajectory');

figure;
subplot(4,1,1);
plot(t,vx,'.'); hold on;
plot(t,vy,'.');
legend('vx [m/s]','vy [m/s]');
grid minor;

subplot(4,1,2);
plot(t,w*180/pi,'.'); hold on;
legend('\omega [deg/s]');
grid minor;

subplot(4,1,3);
plot(t,phi*180/pi,'.'); hold on;
legend('phi [deg]');
grid minor;

subplot(4,1,4);
plot(t,PTG_trajIndex,'.'); hold on;
plot(t,PTG_index+1,'.'); 
legend('PTH trajectory Index [deg/s]', 'PTG index');
grid minor;

end

