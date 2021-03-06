% clear 
% syms x [13,1]
% syms u [4,1]
% F = tailsitter_sym(x,u);
% F = [F; 0;0;0;0];
% xi = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 u1 u2 u3 u4];
% syms dt
% p=3;
% [Phi,Psi_p,JPhi,JPhiT] = compute_Phi_and_JPhi(p,F,xi,dt);
% save('tailsitter_phi.mat','p','Phi','Psi_p','JPhi','JPhiT');
%%
clear
close all
euler0 = [0 pi/2 0];
q0 = eul2quat(euler0)';

pos0 = [0; 0; -2];
v0 = [0; 0; 0];
omega0 = [0; 0; 0];
xeq = fsolve(@nonlinear_dynamics_eq, [omega0; q0; v0; pos0; 0; 0; 240; 240]);
%INPUT REFERENCE ATTITUDE HERE FOR THE DISTURBANCE BELOW
q0_1 = eul2quat(deg2rad([0 20 0])); 
% yaw, pitch, roll. doesn't include pitch at 90
%%
q0 = eul2quat([0 0 0])';
ueq = xeq(14:17)';
x0 = [[0; 0; 0]; q0; v0; pos0];
tf = 20;
dt = 0.005;
t = 0:dt:tf;
disturbance = 1:length(t);
q_ref = q0'.*ones(length(t),4);
q_ref(disturbance,:) = q0_1.*ones(length(disturbance),4);

x_ode = x0;
x_adap = x0;
u_ode = 0*ueq';
u_adap = u_ode;
Kq = 10;
Kqi = 50;
Kp = 10;
Kpi = 100;
Kr = 10;
Kri = 5;
Kt = -5;
Kti = -100;
Kvz = -0*20;
Kvzi = -0*10;

KP = diag([Kt Kp Kq Kr]);
KI = diag([Kti Kpi Kqi Kri]);
M_motor = [1 1; 1 -1];
M_flap = [-1 1; -1 -1];
flap_max = 20;
flap_rate = 400;

% load tailsitter_phi.mat
for i = 1:length(t)
%     if abs(u_adap(1:2,i)) > flap_max
%         u_adap(1:2,i) = flap_max*sign(u_adap(1:2,i));
%     end
%     [~, x_adap_out] = adaptive_taylor(p,Phi,Psi_p,[0 dt],[x_adap(:,i);u_adap(:,i)]);
%     x_adap(:,i+1) = x_adap_out(end,1:13)' + perturbation(:,i);
%     [~,accel_adap(:,i)] = tailsitter(-1,x_adap(:,i),u_adap(:,i)+ueq');
%     y_adap(:,i) = [accel_adap(1,i); x_adap(1:3,i)];
%     y_adap_int(:,i) = dt*trapz(y_adap,2);
%     cmd_adap =  -KP*y_adap(:,i) - KI*y_adap_int(:,i);
%     motor_cmd_adap = M_motor*[cmd_adap(1); cmd_adap(4)];
%     flap_cmd_adap = M_flap*[cmd_adap(3); cmd_adap(2)];
%     u_adap(:,i+1) = [flap_cmd_adap; motor_cmd_adap];
    
    
    [~,ode_out]  = ode45(@(t,x) tailsitter(t,x,u_ode(:,i)+ueq'),[0 dt],x_ode(:,i));
    x_ode(:,i+1) = ode_out(end,:)';
    axis_err(:,i) = quat_feedback(x_ode(4:7,i)',q_ref(i,:));
    [~,accel_ode(:,i)] = tailsitter(t(i),x_ode(:,i),u_ode(:,i)+ueq');
    accel_ode(:,i) = quat2dcm([-1 0 1 0])*quat2dcm(x_ode(4:7,i)')'*accel_ode(:,i);
    y_ode(:,i) = [accel_ode(3,i)+9.81; x_ode(1:3,i)];
    y_ode_int = dt*trapz(y_ode,2);
    vel(:,i) = quat2dcm([-1 0 1 0])*quat2dcm(x_ode(4:7,i)')'*x_ode(8:10,i);
    vel_int = dt*trapz(vel,2);
    axis_err_int = dt*trapz(axis_err,2);
    cmd_ode =  -KP*(y_ode(:,i) +10*[0; axis_err(:,i)]) - KI*([y_ode_int(1); 10*axis_err_int]);
    cmd_ode(3) = cmd_ode(3) -Kvz*vel(1,i) -Kvzi*vel_int(1);
    motor_cmd_ode = M_motor*[cmd_ode(1); cmd_ode(4)];
    flap_cmd_ode = M_flap*[cmd_ode(3); cmd_ode(2)];
    u_ode(:,i+1) = [flap_cmd_ode; motor_cmd_ode];

    if abs(u_ode(1,i+1) - u_ode(1,i)) > flap_rate*dt
        u_ode(1,i+1) = u_ode(1,i) + flap_rate*dt*sign(u_ode(1,i+1) - u_ode(1,i));
    end
    
    if abs(u_ode(2,i+1) - u_ode(2,i)) > flap_rate*dt
        u_ode(2,i+1) = u_ode(2,i) + flap_rate*dt*sign(u_ode(2,i+1) - u_ode(2,i));
    end
    
    if abs(u_ode(1,i+1)) > flap_max
        u_ode(1,i+1) = flap_max*sign(u_ode(1,i+1));
    end
    if abs(u_ode(2,i+1)) > flap_max
        u_ode(2,i+1) = flap_max*sign(u_ode(2,i+1));
    end
    
    elevator(i) = cmd_ode(3);
    aileron(i) = cmd_ode(4);
end
x_ode = (x_ode(:,1:end-1))';
% x_adap = (x_adap(:,1:end-1))';
x_adap = NaN*x_ode;

u_ode = (u_ode(:,1:end-1))' + ueq;
% u_adap = (u_adap(:,1:end-1))' + ueq;
u_adap = NaN*u_ode;

figure
subplot(2,2,1)
plot(t,u_ode(:,1),t, u_adap(:,1), ':')
ylabel('f left (deg)')
subplot(2,2,2)
plot(t,u_ode(:,2),t, u_adap(:,2), ':')
ylabel('f right (deg)')
subplot(2,2,3)
plot(t,u_ode(:,3),t, u_adap(:,3), ':')
ylabel('n left (rev/s)')
subplot(2,2,4)
plot(t,u_ode(:,4),t, u_adap(:,4), ':')
ylabel('n right (rev/s)')
sgtitle('u')

euler_ode = quat2eul(x_ode(:,4:7));
euler_adap = quat2eul(x_adap(:,4:7));
euler_ode = rad2deg(unwrap(euler_ode));
euler_adap = rad2deg(unwrap(euler_adap));

figure
subplot(3,1,1)
plot(t,euler_ode(:,3),t,euler_adap(:,3),':')
ylabel('\phi (deg)')
subplot(3,1,2)
plot(t,90+euler_ode(:,2),t,euler_adap(:,2),':')
ylabel('\theta (deg)')
subplot(3,1,3)
plot(t,euler_ode(:,1),t,euler_adap(:,1),':')
ylabel('\psi (deg)')
sgtitle('Euler angles')

figure
subplot(3,1,1)
plot(t,x_ode(:,1),t,x_adap(:,1),':')
ylabel('P (rad/s)')
subplot(3,1,2)
plot(t,x_ode(:,2),t,x_adap(:,2),':')
ylabel('Q(rad/s)')
subplot(3,1,3)
plot(t,x_ode(:,3),t,x_adap(:,3),':')
ylabel('R (rad/s)')
sgtitle('PQR')

figure
subplot(3,1,1)
plot(t,x_ode(:,8),t,x_adap(:,8),':')
ylabel('V_x (m/s)')
subplot(3,1,2)
plot(t,x_ode(:,9),t,x_adap(:,9),':')
ylabel('V_y (m/s)')
subplot(3,1,3)
plot(t,x_ode(:,10),t,x_adap(:,10),':')
ylabel('V_z (m/s)')
sgtitle('V')

S_w = 0.08;
rho = 1.225;

ram_drag = [-sign(x_ode(:,8))*0.5*rho.*x_ode(:,8).^2*S_w*0.1, ...
    -sign(x_ode(:,9))*0.5*rho.*x_ode(:,9).^2*S_w*0.1, ...
    -sign(x_ode(:,10))*0.5*rho.*x_ode(:,10).^2*S_w*1.3];
figure
plot(t,ram_drag)
title('Drag in vehicle frame')
legend({'X', 'Y', 'Z'})

figure
plot(t,accel_ode)
legend({'X', 'Y', 'Z'})
title('accelerations in vehicle frame')

figure
subplot(3,1,1)
plot(t,x_ode(:,11),t,x_adap(:,11),':')
ylabel('x (m)')
subplot(3,1,2)
plot(t,x_ode(:,12),t,x_adap(:,12),':')
ylabel('y (m)')
subplot(3,1,3)
plot(t,x_ode(:,13),t,x_adap(:,13),':')
ylabel('z (m)')
sgtitle('x')
%%
yout = [unwrap(quat2eul(x_ode(:,4:7))) -x_ode(:,end) x_ode(:,4:7)];
% u_interp = interp1(t, u, t, 'linear');
stick = [elevator', aileron'];
tout = t';
save('test.mat', 'yout', 'stick', 'tout');

att_traj = x_ode(:,4:7);
pos_traj = x_ode(:,11:13);

%%
% run_animation_tailsitter
position_animation