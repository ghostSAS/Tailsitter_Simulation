
euler0 = [0 pi/2 0];
q0 = eul2quat(euler0)';

pos0 = [0; 0; -10];
v0 = [0; 0; 0];
omega0 = [0; 0; 0];

x0 = [omega0; q0; v0; pos0];
tf = 5;
t_vec = 0:0.01:tf;
u = zeros(1,4).*t_vec';
u(:,3:4) = 76.5*ones(length(t_vec),2);
u(:,1:2) = 0*ones(length(t_vec),2);
[t,x] = ode45(@(t,x) nonlinear_dynamics_hover(t,x,u,t_vec), [0 tf], x0);

euler = quat2eul(x(:,4:7));
% figure
% subplot(3,1,1)
% plot(t,euler(:,1))
% subplot(3,1,2)
% plot(t,euler(:,2))
% subplot(3,1,3)
% plot(t,euler(:,3))

figure
subplot(3,1,1)
plot(t,x(:,11))
subplot(3,1,2)
plot(t,x(:,12))
subplot(3,1,3)
plot(t,x(:,13))





