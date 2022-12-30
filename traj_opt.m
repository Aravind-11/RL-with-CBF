close all;
clear;
clc;

dt = 0.1;                                       % Sampling time
n = 3;                                         % horizon length
mpc_sz = 3;                                     % Total number of steps taken at a time 

% x symbolizes x direction, y for y direction, v is for velocity and the
% number is for the number of the robot

% Acceleration
vx1_dot = 0.5;
vx2_dot = 0.5;
vy1_dot = 0.5;
vy2_dot = 0.5;

% Position
x1 = 0;
y1 = 0;
x2 = 50;
y2 = 50;

% Velocity
vx1 = 0;
vx2 = 0;
vy1 = 0;
vy2 = 0;

% Goal position
x1_f = 50;
y1_f = 50;
x2_f = 0;
y2_f = 0;

% Robot trajectory
traj_x1 = [];
traj_y1 = [];
traj_x2 = [];
traj_y2 = [];

v_x1 = [];
v_x2 = [];
v_y1 = [];
v_y2 = [];

u1_x = [];
u1_y = [];
u2_x = [];
u2_y = [];

% Bound on the control
lb = -5*ones(1, 4*n);
ub = 5*ones(1, 4*n);

%u0 = -5 + (10)*rand(1,4);
u0 = zeros(1, 4*n);
x0 = [x1, y1, vx1, vy1, x2, y2, vx2, vy2];               % Initial state
x_f = [x1_f, x2_f, y1_f, y2_f];                          % Final state

options = optimoptions('fmincon', 'MaxFunctionEvaluations', 1e6, 'MaxIterations', 1e4);
OPTIONS =optimoptions('fmincon','Algorithm','sqp');

while(true)
     
   obj = @(u)kinematics(x0, u, dt, x_f, n);                                         % con = control input
   [y, yval] = fmincon(obj,u0,[],[],[],[],lb,ub,@(u)non_lin(u, x0, n),options);
   u0 = y;
%    u0= -5 + 10*rand(1,4*n);

   % updating the next position of both robots
   
   x1(1) = x0(1);
   y1(1) = x0(2);
   vx1(1) = x0(3);
   vy1(1) = x0(4);
   
   x2(1) = x0(5);
   y2(1) = x0(6);
   vx2(1) = x0(7);
   vy2(1) = x0(8);
   
%    Control
for i = 1:1
   
   u1 = [y(4*i - 3), y(4*i - 2)];
   u2 = [y(4*i - 1), y(4*i)];
   
   k1 = [vx1(i), vy1(i); 0, 0] + [0; 1]*u1;
   k2 = [vx2(i), vy2(i); 0, 0] + [0; 1]*u2;

   vx1(i) = k1(1,1);
   vy1(i) = k1(1,2);

   vx1_dot(i) = k1(2,1);
   vy1_dot(i) = k1(2,2);

   vx2(i) = k2(1,1);
   vy2(i) = k2(1,2);

   vx2_dot(i) = k2(2,1);
   vy2_dot(i) = k2(2,2);
   
   x1(i+1) = x1(i) + vx1(i)*dt;
   y1(i+1) = y1(i) + vy1(i)*dt;

   x2(i+1) = x2(i) + vx2(i)*dt;
   y2(i+1) = y2(i) + vy2(i)*dt;

   vx1(i+1) = vx1(i) + vx1_dot(i)*dt;
   vy1(i+1) = vy1(i) + vy1_dot(i)*dt;
   vx2(i+1) = vx2(i) + vx2_dot(i)*dt;
   vy2(i+1) = vy2(i) + vy2_dot(i)*dt;
   
end
   
   traj_x1 = [traj_x1, x1(1:mpc_sz-1)];
   traj_y1 = [traj_y1, y1(1:mpc_sz-1)];
   traj_x2 = [traj_x2, x2(1:mpc_sz-1)];
   traj_y2 = [traj_y2, y2(1:mpc_sz-1)];
   
   v_x1 = [v_x1, vx1(1:mpc_sz-1)];
   v_y1 = [v_y1, vy1(1:mpc_sz-1)];
   v_x2 = [v_x2, vx2(1:mpc_sz-1)];
   v_y2 = [v_y2, vy2(1:mpc_sz-1)];
   
   disp(traj_x1(end));
   disp(traj_y1(end));
   disp(traj_x2(end));
   disp(traj_y2(end));
   
   x0 = [traj_x1(end), traj_y1(end), v_x1(end), v_y1(end), traj_x2(end), traj_y2(end), v_x2(end), v_y2(end)];       % Initial state
   
   dis_brk1 = (traj_x1(end) - x1_f)^2 + (traj_y1(end) - y1_f)^2;            % Distance for robot1
   dis_brk2 = (traj_x2(end) - x2_f)^2 + (traj_y2(end) - y2_f)^2;            % Distance for robot2
   dis_brk = dis_brk1 + dis_brk2;
   disp(yval);
   
   for i = 1:mpc_sz-1
      % for i = 1:1
       
   u1_x = [u1_x, y(4*i - 3)];
   u1_y = [u1_y, y(4*i - 2)];
   u2_x = [u2_x, y(4*i - 1)];
   u2_y = [u2_y, y(4*i)];
   
   end
   
   if(dis_brk1 < 10 && dis_brk2 < 10) % for the last iteration
%    if(yval < 1e-4)
       
%        traj_x1 = [traj_x1, x1(mpc_sz:n)];
%        traj_y1 = [traj_y1, y1(mpc_sz:n)];
%        traj_x2 = [traj_x2, x2(mpc_sz:n)];
%        traj_y2 = [traj_y2, y2(mpc_sz:n)];
%        
%        v_x1 = [v_x1, vx1(mpc_sz:n)];
%        v_y1 = [v_y1, vy1(mpc_sz:n)];
%        v_x2 = [v_x2, vx2(mpc_sz:n)];
%        v_y2 = [v_y2, vy2(mpc_sz:n)];
%        
%        for i = mpc_sz:n
%        
%        u1_x = [u1_x, y(4*i - 3)];
%        u1_y = [u1_y, y(4*i - 2)];
%        u2_x = [u2_x, y(4*i - 1)];
%        u2_y = [u2_y, y(4*i)];
% 
%        end
       
       disp(traj_x1(end));
       disp(traj_y1(end));
       disp(traj_x2(end));
       disp(traj_y2(end));
       break;
   end
   
end

%%
sz = size(traj_x1);
sz = linspace(0,sz(2)*dt,sz(2));

% figure;
% plot(sz, traj_x1);
% hold on;
% plot(sz, traj_x2);
% legend('Robot 1', 'Robot 2');
% xlabel('t');
% ylabel('x');
% hold off;

% figure;
% plot(sz, traj_y1);
% hold on;
% plot(sz, traj_y2);
% legend('Robot 1', 'Robot 2');
% xlabel('t');
% ylabel('y');
% hold off;

figure;
plot(traj_x1, traj_y1);
hold on;
plot(traj_x2, traj_y2);
legend('Robot 1', 'Robot 2');
xlabel('x');
ylabel('y');
title('Position')
hold off;

% figure;
% title('Velocity')
% plot(v_x1, v_y1);
% hold on;
% plot(v_x2, v_y2);
% legend('Robot 1', 'Robot 2');
% xlabel('Vx');
% ylabel('Vy');
% hold off;

v1 = sqrt(v_x1.*v_x1 + v_y1.*v_y1);
v2 = sqrt(v_x2.*v_x2 + v_y2.*v_y2);

figure;
plot(sz, v1);
hold on;
plot(sz, v2);
legend('Robot 1', 'Robot 2');
xlabel('t');
ylabel('v');
title('Velocity')
hold off;

dis = sqrt((traj_x1 - traj_x2).^2 + (traj_y1 - traj_y2).^2);

figure;
plot(sz, dis);
xlabel('t');
ylabel('D_s');
title('Distance')
hold off;

% figure;
% plot(sz, u1_x);
% hold on;
% plot(sz, u1_y);
% legend('t', 'y');
% hold off;
% 
% figure;
% plot(sz, u2_x);
% hold on;
% plot(sz, u2_y);
% legend('t', 'y');
% hold off;

figure;
plot(sz, u1_x);
hold on;
plot(sz, u1_y);
plot(sz, u2_x);
plot(sz, u2_y);
legend('u_{1x}', 'u_{1y}', 'u_{2x}', 'u_{2y}');
xlabel('t');
ylabel('u');
title('Control');
hold off;