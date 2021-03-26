%% geometry and mass properties
clear
tf = 4;
dt = 0.005
m = 0.3; % mass [kg]
g = 9.81; % acceleration due to gravity [m/s^2]
Jxx = 0.00447; % moment of inertia about x axis [kg-m^2]
Jyy = 0.00065; % moment of inertia about y axis [kg-m^2]
Jzz = 0.00512; % moment of inertia about z axis [kg-m^2]
Jxz = -0.00001; % cross-product of inertia xz [kg-m^2]
J = [Jxx 0 -Jxz; 0 Jyy 0; -Jxz 0 Jzz];
S_w = 0.0828; % wing area [m^2]

c_bar = 0.142; % mean aerodynamic chord [m]
b = 0.6; % wing span [m]
D = 0.12; % propeller diameter [m]
prop_arm = 0.15; % distance from x axis to propeller thrust axis [m]
% y_mac = 0.1643; % spanwise location of aerodynamic center [m]

% aerodynamic parameters
rho = 1.225; % air density [kg/m^3]
% CL_0 = 0.147; % lift coefficient at 0 angle of attack (AoA)
CL_0=0;
CL_f = 0.036; % lift coefficient per flap deflection [1/deg]
CD_f = 0.000762;
% Cm_0 = -0.065; % moment coefficient at 0 AoA
Cm_0 = 0;
Cm_f = -0.0102; % moment coefficient per flap deflection [1/deg]
CT = 0.05; % thrust coefficient
CP = 0.01; % power coefficient
Cl_f = 0.0057;

xeq = fsolve(@nonlinear_dynamics_eq, [zeros(3,1); 1; zeros(11,1); 240; 240]);

motor_eq = xeq(end);

Kq = 10;
Kqi = 50;
Kp = 10;
Kpi = 100;
Kr = 10;
Kri = 5;
Kt = -5;
Kti = -100;
Kvz = -20;
Kvzi = -10;

% KP = diag([Kt Kp Kq Kr]);
% KI = diag([Kti Kpi Kqi Kri]);
M_motor = [1 1; 1 -1];
M_flap = [-1 1; -1 -1];
flap_max = 20;
flap_rate = 400;

%%
u = [out.flaps out.motors];
t = 0:dt:tf;
euler = out.euler;
omega = out.omega;
V = squeeze(out.V)';
X = squeeze(out.V)';
quat = out.quat;
accel = squeeze(out.accel)';

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

% S_w = 0.08;
% rho = 1.225;
% 
% ram_drag = [-sign(x(:,8))*0.5*rho.*x(:,8).^2*S_w*0.1, ...
%     -sign(x(:,9))*0.5*rho.*x(:,9).^2*S_w*0.1, ...
%     -sign(x(:,10))*0.5*rho.*x(:,10).^2*S_w*1.3];
% figure
% plot(t,ram_drag)
% title('Drag in vehicle frame')
% legend({'X', 'Y', 'Z'})

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

yout = [unwrap(quat2eul(quat)) -X(:,3) quat];
% u_interp = interp1(t, u, t, 'linear');
% stick = [elevator', aileron'];
stick = 0*[t t];
tout = t';
save('test.mat', 'yout', 'stick', 'tout');

att_traj = quat;
pos_traj = X;


% run_animation_tailsitter
position_animation