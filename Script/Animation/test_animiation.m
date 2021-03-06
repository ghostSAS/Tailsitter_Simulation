%% compute traj
clear all, close all, clc


% euler0 = [0 pi/2 0];
% q0 = eul2quat(euler0)';
% 
% pos0 = [0; 0; -10];
% v0 = [0; 0; 0];
% omega0 = [0; 0; 0];

euler0 = [0 pi/2 0];
q0 = eul2quat(euler0)';

pos0 = [0; 0; 0];
v0 = rand(3,1);
omega0 = rand(3,1)/10;

x0 = [omega0; q0; v0; pos0];
tf = 5;
t_vec = 0:0.01:tf;
u = zeros(1,4).*t_vec';
u(:,3:4) = 76.5*ones(length(t_vec),2);
u(:,1:2) = 0*ones(length(t_vec),2);
[t,x] = ode45(@(t,x) nonlinear_dynamics_hover(t,x,u,t_vec), [0 tf], x0);

att_traj = quat2eul(x(:,4:7));
pos_traj = x(:,11:13);
% figure
% subplot(3,1,1)
% plot(t,euler(:,1))
% subplot(3,1,2)
% plot(t,euler(:,2))
% subplot(3,1,3)
% plot(t,euler(:,3))

figure('Name','position')
subplot(3,1,1)
plot(t,x(:,11))
subplot(3,1,2)
plot(t,x(:,12))
subplot(3,1,3)
plot(t,x(:,13))

figure('Name','attitude')
for i = 1:size(att_traj,2)
    subplot(3,1,i)
    plot(t,att_traj(:,i))
end



%% animation
close all, clc

time_delay = .01;


% airplane shape
shape.body.vertex = [[1.5;0;0], [-1;0;0]];
shape.body.width = 8;
shape.body.color = 'r';
shape.wing.vertex = [[-.3;.7;0],[-.3;-.7;0],[.3;-.7;0],[.3;.7;0],[-.3;.7;0]];
shape.wing.width = 4;
shape.wing.color = 'g';


% plot
viewRange = compute_viewRange(pos_traj);
figure
for i=1:size(pos_traj,1)
    pos = pos_traj(i,:);
    att = att_traj(i,:);
    
    T_shape = trans(pos, att, shape);
    name = fieldnames(shape);
    for j = 1:length(name)
        part = T_shape.(name{j}).vertex;
        plot3(part(1,:), part(2,:), part(3,:), shape.(name{j}).color,'LineWidth', shape.(name{j}).width);
        hold on
    end 
    axis equal
    grid on
    axis(viewRange);
    view([44 16])
    xlabel('x'); ylabel('y'); zlabel('z')
    drawnow
    
    pause(time_delay)
    hold off
    
end


function range = compute_viewRange(traj)
    range = [min(traj(:,1)) max(traj(:,1)) min(traj(:,2)) max(traj(:,2)) min(traj(:,3)) max(traj(:,3))];
    midPt = [min(traj(:,1))+max(traj(:,1)) min(traj(:,1))+max(traj(:,1)) ...
                min(traj(:,2))+max(traj(:,2)) min(traj(:,2))+max(traj(:,2)) ...
                min(traj(:,3))+max(traj(:,3)) min(traj(:,3))+max(traj(:,3))]/2;
            
    % make sure there are enough space to view 
    dif = range - midPt;
    dif(abs(dif)<1 .* [1 0 1 0 1 0]) = -1;
    dif(abs(dif)<1 .* [0 1 0 1 0 1]) = 1;
    dif(dif<0) = dif(dif<0)-2;
    dif(dif>0) = dif(dif>0)+2;
    
    range = midPt+dif;
    
end


function shape = trans(pos, att, shape)
%     x = pos(1); y = pos(2); z = pos(3);
    c = att(1); b = att(2); a = att(3); 
    
    T_rotate = [ cos(a)*cos(b), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), sin(a)*sin(c) + cos(a)*cos(c)*sin(b);
                 cos(b)*sin(a), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c);
                       -sin(b),                        cos(b)*sin(c),                        cos(b)*cos(c)];
       
    T_move = pos(:);
    
    % compute the rotated airplane
    name = fieldnames(shape);
    for i = 1:length(name)
        part = shape.(name{i}).vertex;
        T_part = zeros(size(part));
        for j = 1:size(part,2)
            T_part(:,j) = T_rotate*part(:,j)+T_move;
        end
        shape.(name{i}).vertex = T_part;
    end

end
    
