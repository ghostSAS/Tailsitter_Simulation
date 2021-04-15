clear
roll_angle_cmd = 0;
pitch_angle_cmd = 20;
yaw_angle_cmd = 0;

u_cmd = 0;
v_cmd = 0;
w_cmd = 0;

x_cmd = 0;
y_cmd = 0;
h_cmd = 2;

tf = 20;

simulink_setup;
sim('tailsitter_simulation_cascaded');
out = ans;
simulink_plotter;
%%
% run_animation_tailsitter
%%
position_animation