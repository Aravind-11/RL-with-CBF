function [cost] = kinematics(x0, con, dt, x_f, n)

x1 = x0(1);
y1 = x0(2);
vx1 = x0(3);
vy1 = x0(4);

x2 = x0(5);
y2 = x0(6);
vx2 = x0(7);
vy2 = x0(8);

x1_f = x_f(1);
x2_f = x_f(2);
y1_f = x_f(3);
y2_f = x_f(4);

kp = 0.04;
kd = 0.5;

u1 = [];
u2 = [];
%u1_bar = [];
%u2_bar = [];

u1_bar = [0,0];
u2_bar = [0,0];

cost = 0;

for i = 1:n

u1 = [con(4*i - 3), con(4*i - 2)];
u2 = [con(4*i - 1), con(4*i)];

u1_bar = [-kp*(x1(i) - x1_f) - kd*vx1(i), -kp*(y1(i) - y1_f) - kd*vy1(i)];
u2_bar = [-kp*(x2(i) - x2_f) - kd*vx2(i), -kp*(y2(i) - y2_f) - kd*vy2(i)];

% Robot kinematics
k1 = [vx1(i), vy1(i); 0, 0] + [0; 1]*u1_bar;
k2 = [vx2(i), vy2(i); 0, 0] + [0; 1]*u2_bar;

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

%u1_bar = [-kp*(x1(i+1) - x1_f) - kd*vx1(i+1), -kp*(y1(i+1) - y1_f) - kd*vy1(i+1)];
%u2_bar = [-kp*(x2(i+1) - x2_f) - kd*vx2(i+1), -kp*(y2(i+1) - y2_f) - kd*vy2(i+1)];

 cost = cost + (norm(u1 - u1_bar))^2 + (norm(u2 - u2_bar))^2;
%cost = (norm(u1 - u1_bar))^2 + (norm(u2 - u2_bar))^2;

end

end