function [c, ceq] = non_lin(u, x0, n)

% Non-linear constraints
dt=0.1;
alpha1 = 5;
alpha2 = 5;
Ds = 5;
eps = 1e-5;

x1 = x0(1);
y1 = x0(2);
vx1 = x0(3);
vy1 = x0(4);

x2 = x0(5);
y2 = x0(6);
vx2 = x0(7);
vy2 = x0(8);

c = [];

for i = 1:n
    
u1 = [u(4*i - 3), u(4*i - 2)];
u2 = [u(4*i - 1), u(4*i)];

vx1=vx1+u1(1)*dt;
vy1=vy1+u1(2)*dt;
vx2=vx2+u2(1)*dt;
vy2=vy2+u2(2)*dt;

x1=x1+vx1*dt;
y1=y1+vy1*dt;
x2=x2+vx2*dt;
y2=y2+vy2*dt;

delta_p = [x1, y1] - [x2, y2];
delta_v = [vx1, vy1] - [vx2, vy2];
norm_p = norm(delta_p);

a12 = (sqrt(2*(alpha1 + alpha2)*(norm_p - Ds)) + (delta_p*delta_v')/(norm_p ));
%a12 = (sqrt(2*(alpha1 + alpha2)*(norm_p - Ds)*(norm_p - Ds)) + (delta_p*delta_v')/(norm_p ))
h12=a12-Ds;
gamma=1;
b12 = (norm_p*gamma*(h12^5) + 0.5*(2*(alpha1 + alpha2)*delta_p*delta_v')/(sqrt(2*(alpha1 + alpha2)*(norm_p - Ds)) ) ...
      + delta_v*delta_v' - ((delta_p*delta_v')^2/(norm_p^2 + eps)));

%c = [c; -delta_p*u1' - alpha1*b12/(alpha1 + alpha2); delta_p*u2' - alpha2*b12/(alpha1 + alpha2)];
c=[c;-delta_p*u1' + delta_p*u2' - b12];

end

% c = [-delta_p*u1' + delta_p*u2' - b12];

%c=-delta_p'*delta_v-b12;
% Inequality constraints
ceq = [];

end