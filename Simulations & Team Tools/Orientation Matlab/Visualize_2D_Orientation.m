clear
clc
format compact

%Create a unit circle
theta = linspace(0, 2 * pi, 100);  % Create 100 equally spaced points around the unit circle
x = cos(theta);  % Calculate x-coordinates
y = sin(theta);  % Calculate y-coordinates

origin = [0,0];
e_x = [0, 1];
e_y = [-1, 0];

psi = 145; % in degrees
offset = atan2(e_x(2),e_x(1))*180/pi; %how the inertial frame is off the normal x,y axis

b_x = [cosd(psi+offset), sind(psi+offset)];
b_y = [-sind(psi+offset), cosd(psi+offset)];

servo1_offset = 45;
servo2_offset = -45;
servo1 = [cosd(servo1_offset+offset+psi), sind(servo1_offset+offset+psi)];
servo2 = [cosd(servo2_offset+offset+psi), sind(servo2_offset+offset+psi)];

gamma = 45; %in degrees from inerital
goal = [cosd(gamma+offset), sind(gamma+offset)];

delta = find_delta(psi, gamma);
[servo1_mag, servo2_mag] = move_servo(delta, servo1_offset, servo2_offset);

servo1 = servo1 * servo1_mag;
servo2 = servo2 * servo2_mag;

%%Graph the results

figure;
hold on;

%Graph Unit Vectors
quiver(origin(1), origin(2), e_x(1), e_x(2), 1, 'b')
quiver(origin(1), origin(2), e_y(1), e_y(2), 1, 'b')

text(e_x(1)*1.1, e_x(2)*1.1, 'e_x', 'FontSize', 12);
text(e_y(1)*1.1, e_y(2)*1.1, 'e_y', 'FontSize', 12);

%Graph Body Frame Vectors
quiver(origin(1), origin(2), b_x(1), b_x(2), 1, 'g')
quiver(origin(1), origin(2), b_y(1), b_y(2), 1, 'g')

text(b_x(1)*1.1, b_x(2)*1.1, 'b_x', 'FontSize', 12);
text(b_y(1)*1.1, b_y(2)*1.1, 'b_y', 'FontSize', 12);

%Graph Goal Vector
quiver(origin(1), origin(2), goal(1), goal(2), 1, 'r')

text(goal(1)*1.1, goal(2)*1.1, 'goal', 'FontSize', 12);

%Graph Servo Velocity vector
quiver(origin(1), origin(2), servo1(1), servo1(2), 1, 'k')
quiver(origin(1), origin(2), servo2(1), servo2(2), 1, 'k')

text(servo1(1)*1.1, servo1(2)*1.1, 'S1', 'FontSize', 12);
text(servo2(1)*1.1, servo2(2)*1.1, 'S2', 'FontSize', 12);

%Graph Angles between vectors
psi_r = .1;
psi_angle = linspace(offset*(pi/180),(psi+offset)*(pi/180), 100);
psi_x = psi_r*cos(psi_angle); psi_y = psi_r*sin(psi_angle);

gamma_r = .15;
gamma_angle = linspace(offset*(pi/180),(gamma+offset)*(pi/180), 100);
gamma_x = gamma_r*cos(gamma_angle); gamma_y = gamma_r*sin(gamma_angle);

delta_r = .2;
delta_angle = linspace((offset+gamma)*(pi/180),(psi+offset)*(pi/180), 100);
delta_x = delta_r*cos(delta_angle); delta_y = delta_r*sin(delta_angle);

plot(x, y, 'k', psi_x, psi_y, 'g', gamma_x, gamma_y, 'r', delta_x, delta_y, 'm');
legend('e_x', 'e_y', 'b_x', 'b_y', 'goal', 'Servo1 Velo', 'Servo2 Velo', 'Unit Circle', '\psi', '\gamma', '\delta')
axis([-1.5 1.5 -1.5 1.5]);
axis equal;
title('Servo Velo Vector Visualization');
xlabel('x');
ylabel('y');