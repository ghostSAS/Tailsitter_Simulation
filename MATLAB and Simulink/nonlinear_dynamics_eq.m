function xdot = nonlinear_dynamics_eq(x)
omega = x(1:3); % angular rates
quat = x(4:7); % quaternion
V = x(8:10); % translational velocities

f_left = x(14); % left flap [deg]
f_right = x(15); % right flap [deg]
n_left = x(16); % left prop speed [Hz] or [1/s]
n_right = x(17); % right prop [Hz] or [1/s]

% geometry and mass properties
m = 0.3; % mass [kg]
g = 9.81; % acceleration due to gravity [m/s^2]
Jxx = 0.00447; % moment of inertia about x axis [kg-m^2]
Jyy = 0.00065; % moment of inertia about y axis [kg-m^2]
Jzz = 0.00512; % moment of inertia about z axis [kg-m^2]
Jxz = -0.00001; % cross-product of inertia xz [kg-m^2]
J = [Jxx 0 -Jxz; 0 Jyy 0; -Jxz 0 Jzz];
S_w = 0.0828; % wing area [m^2]

c_bar = 0.142; % mean aerodynamic chord [m]
D = 0.12; % propeller diameter [m]
prop_arm = 0.15; % distance from x axis to propeller thrust axis [m]
y_mac = 0.1643; % spanwise location of aerodynamic center [m]

% aerodynamic parameters
rho = 1.225; % air density [kg/m^3]
CL_0 = 0.147; % lift coefficient at 0 angle of attack (AoA)
CL_0 = 0;
CL_f = 0.036; % lift coefficient per flap deflection [1/deg]
CD_f = 0.000762;
Cm_0 = -0.065; % moment coefficient at 0 AoA
Cm_0 = 0;
Cm_f = -0.0102; % moment coefficient per flap deflection [1/deg]
CT = 0.05; % thrust coefficient
CP = 0.01; % power coefficient

% most likely unnecessary values
Cl_f = 0.0057; % roll coefficient per flap deflection [1/deg]
b = 0.6; % wing span [m]

T_left = CT*rho*D^4*n_left^2; % left thrust [N]
T_right = CT*rho*D^4*n_right^2; % right thrust [N]

% v_left = sqrt(8*T_left/(rho*pi*D^2)); % velocity over left wing [m/s]
% v_right = sqrt(8*T_right/(rho*pi*D^2)); % velocity over right wing [m/s]

S_w = S_w/3;
q_S_left = 4*T_left/(pi*D^2); % dynamic pressure over left wing [N/m^2]
q_S_right = 4*T_right/(pi*D^2); % dynamic pressure over right wing [N/m^2]

lift_left = q_S_left*S_w/2*(CL_0 + CL_f*f_left);
lift_right = q_S_right*S_w/2*(CL_0 + CL_f*f_right);

drag_left = q_S_left*S_w/2*CD_f*abs(f_left);
drag_right = q_S_right*S_w/2*CD_f*abs(f_right);

L_P = CP*rho*D^5/(2*pi)*(n_right^2 - n_left^2); % propeller roll [N-m]
N_P = prop_arm*(T_left - T_right); % yaw due to thrust [N-m]
    
% define positive flap as downwards deflection
% positive left flap deflection results in a postive roll
% L_A = (lift_left - lift_right)*y_mac; % roll from aerodynamics [N-m]
L_A = q_S_left*S_w/2*b*Cl_f*f_left - ...
    q_S_right*S_w/2*b*Cl_f*f_right; % roll from aerodynamics [N-m]
M_A = q_S_left*S_w/2*c_bar*(Cm_0 + Cm_f*f_left) + ...
    q_S_right*S_w/2*c_bar*(Cm_0 + Cm_f*f_right); % pitch from aerodynamics [N-m]

% cross product matrix trick: cross_omega*x is equivalent to cross(omega,x)
cross_omega = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];

T_eb = quat2dcm(quat'); % coordinate transformation from earth to body

F_A = [-(drag_left + drag_right); 0; -(lift_left + lift_right)]; % aerodynamic force (lift) [N]

F_P = [T_left + T_right; 0; 0]; % propulsive force [N]

F_G = T_eb*[0; 0; m*g]; % force of gravity [N]

omega_dot = J\(-cross_omega*J*omega + [L_A + L_P; M_A; N_P]); % omega diff eq

V_dot = -cross_omega*V + 1/m*(F_P + F_A + F_G); % velocity diff eq

pos_dot = T_eb'*V; % position diff eq

quat_dot = 0.5*[0 -omega'; omega cross_omega]*quat; % quaternion kinematics diff eq

xdot = [omega_dot; quat_dot; V_dot; pos_dot]; % combine derivatives and send back as xdot

end