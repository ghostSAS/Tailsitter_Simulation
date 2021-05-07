function [xdot, accel] = tailsitter(t,x,u)

omega = x(1:3); % angular rates
P = omega(1);
Q = omega(2);
R = omega(3);
quat = x(4:7); % quaternion
V = x(8:10); % translational velocities
V_T = norm(V); % total velocity magnitude
alpha = arctan2(V(3),V(1));
beta = arcsin(V(2),norm(V));
%% geometry and mass properties
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

%% aerodynamic parameters
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
% Cl_b = -0.0152;
Cl_b = 0;
eff = 0.9929;
AR = 4.348;7; % roll coefficient per flap deflection [1/deg]

%% Define inputs
f_left = u(1); % left flap [deg]
f_right = u(2); % right flap [deg]
n_left = u(3); % left prop speed [Hz] or [1/s]
n_right = u(4); % right prop [Hz] or [1/s]

%% Calculations
T_left = CT*rho*D^4*n_left^2; % left thrust [N]
T_right = CT*rho*D^4*n_right^2; % right thrust [N]

% v_left = sqrt(8*T_left/(rho*pi*D^2)); % velocity over left wing [m/s]
% v_right = sqrt(8*T_right/(rho*pi*D^2)); % velocity over right wing [m/s]
v_left_prop = solve_v_prop(T_left, V_T, rho, D);
v_right_prop = solve_v_prop(T_right, V_T, rho, D);

% S_w = S_w/3; % to account for propeller size
% q_w_left = 4*T_left/(pi*D^2); % dynamic pressure over left wing [N/m^2]
% q_w_right = 4*T_right/(pi*D^2); % dynamic pressure over right wing [N/m^2]
q_bar_left = 1/2*rho*(v_left_prop+V_T)^2;
q_bar_right = 1/2*rho*(v_right_prop+V_T)^2;

lift_left = q_bar_left*S_w/2*(CL_0 + CL_f*f_left + C_from_angle(alpha,CL_a));
lift_right = q_bar_right*S_w/2*(CL_0 + CL_f*f_right);

drag_left = q_bar_left*S_w/2*(CD_f*abs(f_left)+ (CL_0 + CL_f*f_left + C_from_angle(alpha,CL_a))^2/(pi*eff*AR));
drag_right = q_bar_right*S_w/2*(CD_f*abs(f_right)+ (CL_0 + CL_f*f_right + C_from_angle(alpha,CL_a))^2/(pi*eff*AR));

ram_drag(1) = -sign(V(1))*0.5*rho*V(1)^2*S_w*0.1;
ram_drag(2) = -sign(V(2))*0.5*rho*V(2)^2*S_w*0.1;
ram_drag(3) = -sign(V(3))*0.5*rho*V(3)^2*S_w*1.3; % drag from moving in the body z direction [N]

L_P = CP*rho*D^5/(2*pi)*(n_right^2 - n_left^2); % propeller roll [N-m]
N_P = prop_arm*(T_left - T_right); % yaw due to thrust [N-m]

% define positive flap as downwards deflection
% positive left flap deflection results in a postive roll
% L_A = (lift_left - lift_right)*y_mac; % roll from aerodynamics [N-m]
L_A = S_w/2*b*(q_bar_left*Cl_f*f_left - ...
    q_bar_right*Cl_f*f_right + (q_bar_left + q_bar_right)*C_from_angle(beta, Cl_b)); % roll from aerodynamics [N-m]
M_A = q_bar_left*S_w/2*c_bar*(Cm_0 + Cm_f*f_left + C_from_angle(alpha,Cm_a)) + ...
    q_bar_right*S_w/2*c_bar*(Cm_0 + Cm_f*f_right + C_from_angle(alpha,Cm_a)); % pitch from aerodynamics [N-m]

% cross product matrix trick: cross_omega*x is equivalent to cross(omega,x)
cross_omega = [0 -R Q; R 0 -P; -Q P 0];

T_eb = quat2dcm(quat'); % coordinate transformation from earth to body

F_A = ram_drag' + [-(drag_left + drag_right); 0; -(lift_left + lift_right)]; % aerodynamic force (lift) [N]

F_P = [T_left + T_right; 0; 0]; % propulsive force [N]

a_g = T_eb*[-g; 0; 0]; % force of gravity [N]

accel = 1/m*(F_P + F_A);
%% Build xdot
omega_dot = J\(-cross_omega*J*omega + [L_A + L_P; M_A; N_P]); % omega diff eq

V_dot = -cross_omega*V + accel + a_g; % velocity diff eq

pos_dot = quat2dcm([-1 0 1 0])*T_eb'*V; % position diff eq

quat_dot = 0.5*[0 -omega'; omega -cross_omega]*quat; % quaternion kinematics diff eq

xdot = [omega_dot; quat_dot; V_dot; pos_dot]; % combine derivatives and send back as xdot

end

function v_prop = solve_v_prop(T, V_T, rho, D)

fun = @(v) 2*rho*pi*(D/2)^2*v*abs(v+V_T) - T;

v_prop = fzero(fun,1);
end

function C_from_angle = C_from_angle(angle, C_angle_slope)
% angle can be alpha or beta
% C_angle_slope is the coefficient per rad of alpha or beta
% example is CL_a, coefficient of lift per alpha [1/rad]
    abs_angle = abs(angle);
    angle_stall = 25;
    delta_angle = 10;
    if rad2deg(abs_angle) < angle_stall
        C_from_angle = C_angle_slope*abs_angle;
    elseif rad2deg(abs_angle) < angle_stall+delta_angle
        C_from_angle = C_angle_slope*pi/180*angle_stall*(1-180/delta_angle/pi*(abs_angle-angle_stall*pi/180));
    else
        C_from_angle = 0;
    end
    C_from_angle = C_from_angle*sign(angle);
end