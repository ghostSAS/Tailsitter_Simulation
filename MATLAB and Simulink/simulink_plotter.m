%%
close all
u = [out.flaps out.motors];
euler = out.euler;
omega = out.omega;
V = squeeze(out.V_NED)';
X = squeeze(out.pos)';
quat = out.quat;
accel = squeeze(out.accel)';
t = 0:dt:dt*(length(u)-1);

figure
subplot(2,2,1)
plot(t,u(:,1))
ylabel('f left (deg)')
subplot(2,2,2)
plot(t,u(:,2))
ylabel('f right (deg)')
subplot(2,2,3)
plot(t,u(:,3))
ylabel('n left (rev/s)')
subplot(2,2,4)
plot(t,u(:,4))
ylabel('n right (rev/s)')
sgtitle('u')

euler = rad2deg(unwrap(euler));


figure
subplot(3,1,1)
plot(t,euler(:,3))
ylabel('\phi (deg)')
subplot(3,1,2)
plot(t,euler(:,2))
ylabel('\theta (deg)')
subplot(3,1,3)
plot(t,euler(:,1))
ylabel('\psi (deg)')
sgtitle('Euler angles')

figure
subplot(3,1,1)
plot(t,omega(:,1))
ylabel('P (rad/s)')
subplot(3,1,2)
plot(t,omega(:,2))
ylabel('Q(rad/s)')
subplot(3,1,3)
plot(t,omega(:,3))
ylabel('R (rad/s)')
sgtitle('PQR')

figure
subplot(3,1,1)
plot(t,V(:,1))
ylabel('V_x (m/s)')
subplot(3,1,2)
plot(t,V(:,2))
ylabel('V_y (m/s)')
subplot(3,1,3)
plot(t,V(:,3))
ylabel('V_z (m/s)')
sgtitle('V')

figure
plot(t,accel)
legend({'X', 'Y', 'Z'})
title('accelerations in vehicle frame')

figure
subplot(3,1,1)
plot(t,X(:,1))
ylabel('x (m)')
subplot(3,1,2)
plot(t,X(:,2))
ylabel('y (m)')
subplot(3,1,3)
plot(t,X(:,3))
ylabel('z (m)')
sgtitle('x')

figure
plot(X(:,1),X(:,2))
xlabel('x (m)')
ylabel('y (m)')
grid on
title('XY Position')

yout = [unwrap(quat2eul(quat)) -X(:,3) quat];
stick = 0*[t; t];
tout = t';
save('test.mat', 'yout', 'stick', 'tout');

att_traj = quat;
pos_traj = X;