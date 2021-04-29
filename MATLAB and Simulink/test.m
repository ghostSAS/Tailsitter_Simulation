
alpha = deg2rad(-40:40);
cl=0;
for i =1:length(alpha)
    cl(i) = fcn(alpha(i),CL_a);
end
figure
plot(rad2deg(alpha),cl)
ylabel('C_L')
xlabel('\alpha (deg)')
grid on

function CL_from_alpha = fcn(alpha, CL_a)
    abs_alpha = abs(alpha);
    alpha_stall = 25;
    delta_alpha = 10;
    if rad2deg(abs_alpha) < alpha_stall
        CL_from_alpha = CL_a*abs_alpha;
    elseif rad2deg(abs_alpha) < alpha_stall+delta_alpha
        CL_from_alpha = CL_a*pi/180*alpha_stall*(1-180/delta_alpha/pi*(abs_alpha-alpha_stall*pi/180));
    else
        CL_from_alpha = 0;
    end
    CL_from_alpha = CL_from_alpha*sign(alpha);
end

function Cm_from_alpha = fcn1(alpha, Cm_a)
    abs_alpha = abs(alpha);
    alpha_stall = 30;
    delta_alpha = 10;
    if rad2deg(abs_alpha) < alpha_stall
        Cm_from_alpha = Cm_a*abs_alpha;
    elseif rad2deg(abs_alpha) < alpha_stall+delta_alpha
        Cm_from_alpha = Cm_a*pi/180*alpha_stall*(1-180/delta_alpha/pi*(abs_alpha-alpha_stall*pi/180));
    else
        Cm_from_alpha = 0;
    end
    Cm_from_alpha = Cm_from_alpha*sign(alpha);
end