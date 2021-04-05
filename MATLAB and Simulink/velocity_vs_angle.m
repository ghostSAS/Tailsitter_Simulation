pitch_angle = 0:5:45;
velocity = [0 0.3 0.9 1.64 2.57 3.67 4.97 6.48 8.25 10.33];

figure
plot(pitch_angle, velocity)
grid on
title('Velocity vs. Pitch Angle')
xlabel('Pitch angle (deg)')
ylabel('velocity (m/s)')