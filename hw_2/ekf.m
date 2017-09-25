% Homework 2 Autonomous Two-wheele robot EKF
% Jesse Wynn
% September 25, 2017

clc;
clear all;
close all;

% time params
Ts = 0.1;   % sec
t = 0:Ts:20;

% commanded velocity
v_c = 1 + 0.5 * cos(2 * pi * (0.2) * t);
w_c = -0.2 + 2 * cos(2 * pi * (0.6) * t);

% noise params
alpha_1 = 0.1;
alpha_4 = 0.1;
alpha_2 = 0.01;
alpha_3 = 0.01;
alpha_5 = 0;
alpha_6 = 0;

% landmark locations
landmarks = [6, -7 6;   % [x_positions; y_positions]
             4, 8, -4];
         
% initital conditions
x = -5;
y = -3;
theta = pi/2;
first = 0;

% plotting stuff
x_true = zeros(1,length(t));
y_true = zeros(1,length(t));
         
% Task 1: Implement velocity motion model (Table 5.3)
drawRobot(x,y,theta,landmarks,first);
first = 1;
for i=1:length(t)
    pause(0.01)
    v_hat = v_c(i) + randn*sqrt(alpha_1*(v_c(i))^2 + alpha_2*(w_c(i))^2);
    w_hat = w_c(i) + randn*sqrt(alpha_3*(v_c(i))^2 + alpha_4*(w_c(i))^2);
    gamma_hat = randn*sqrt(alpha_5*(v_c(i))^2 + alpha_6*(w_c(i))^2);
    x = x - (v_hat/w_hat)*sin(theta) + (v_hat/w_hat)*sin(theta + w_hat*Ts);
    y = y + (v_hat/w_hat)*cos(theta) - (v_hat/w_hat)*cos(theta + w_hat*Ts);
    theta = theta + w_hat*Ts + gamma_hat*Ts;
    
    %draw the robot
    drawRobot(x,y,theta,landmarks,first)
    
    % save some data for plotting
    x_true(i) = x;
    y_true(i) = y;
end
plot(x_true, y_true)

% Task 2: Simulate range and bearing measurements to landmarks


