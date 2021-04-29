%% geometry and mass properties
dt = 0.005;
m = 0.3; % mass [kg]
g = 9.81; % acceleration due to gravity [m/s^2]
Jxx = 0.00447; % moment of inertia about x axis [kg-m^2]
Jyy = 0.00065; % moment of inertia about y axis [kg-m^2]
Jzz = 0.00512; % moment of inertia about z axis [kg-m^2]
Jxz = -0.00001; % cross-product of inertia xz [kg-m^2]
J = [Jxx 0 -Jxz; 0 Jyy 0; -Jxz 0 Jzz];
S_w = 0.0828; % wing area [m^2]
tf = 10;

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
CL_a = 3.8256; %lift coefficient per alpha [1/rad]
CD_f = 0.000762;
% Cm_0 = -0.065; % moment coefficient at 0 AoA
Cm_0 = 0;
Cm_f = -0.0102; % moment coefficient per flap deflection [1/deg]
Cm_a = -0.3737;
CT = 0.05; % thrust coefficient
CP = 0.01; % power coefficient
Cl_f = 0.0057;
Cl_b = -0.0152;
eff = 0.9929;
AR = 4.348;

xeq = fsolve(@nonlinear_dynamics_eq, [zeros(3,1); 1; zeros(11,1); 240; 240]);

motor_eq = xeq(end);

Kq = 1;
Kqi = 20;
Kp = 1;
Kpi = 10;
Kr = 5;
Kri = 20;
Kt = -5;
Kti = -100;
Ku = -2;
Kui = -2;
Kv = 5;
Kvi = 4;
Kw = 1;
Kd = 0.01;

Kx = 0.2;
Ky = 0.2;
Kh = -1;
Khi = -0.1;

M_motor = [1 1; 1 -1];
M_flap = [-1 1; -1 -1];
flap_max = 20;
flap_rate = 400;

z0 = -5;

load LQR.mat
%states order is: P Q R q1 q2 q3 u v w X Y Z
Q =         diag([0 0 0 100 100 100 0 0 0 1 1 100]);
R = eye(4);
K_lqr = lqr(A_lin, B_lin, Q, R)

