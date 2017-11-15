% Homework 7 Autonomous Two-wheele robot FAST-SLAM 1.0
% Jesse Wynn
% November 14, 2017

clc
clear
close all

% time params
Ts = 0.02;   % sec
t = 0:Ts:20;

% commanded velocity
v_c = 1 + 0.5 * cos(2 * pi * (0.2) * t);
w_c = -0.2 + 2 * cos(2 * pi * (0.6) * t);

% noise params
alpha_1 = 0.1;
alpha_4 = 0.1;
alpha_2 = 0.01;
alpha_3 = 0.01;
alpha_5 = 0.01;
alpha_6 = 0.01;

input_noise = [alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6];

sigma_r = 0.1;
sigma_phi = 0.05;

Q_t = [sigma_r^2, 0;
           0, sigma_phi^2];

% landmark locations
% landmarks = [6, -7, 6, 5, 3, 3, -2, -2, -7, -10;   % [x_positions; y_positions]
%              4, 8, -4, -9, -1, 9, 2, -7, -5, 3];
% landmarks = [6, -7, 6, 5, 3, 3, -2, -2, -7, -10, -7, -2, 9, -1, -5, -1, 10, 1, 9, -9;   % [x_positions; y_positions]
%              4, 8, -4, -9, -1, 9, 2, -7, -5, 3, 0, 6, 9, -2, -9, 9, 0, 5, -9, -9];

% num_landmarks = size(landmarks);
% num_landmarks = num_landmarks(2);
% N = num_landmarks;

min_ = -10;
max_ = 10;
num_landmarks = 50;
N = num_landmarks;
landmarks_x = zeros(1,num_landmarks);
landmarks_y = zeros(1,num_landmarks);
for i = 1:num_landmarks
    landmarks_x(i) = (max_-min_).*rand(1,1) + min_;
    landmarks_y(i) = (max_-min_).*rand(1,1) + min_;
end
landmarks = [landmarks_x; landmarks_y];

% plotting / storing stuff
x_true = zeros(1,length(t));
y_true = zeros(1,length(t));
theta_true = zeros(1,length(t));

x_est = zeros(1,length(t));
y_est = zeros(1,length(t));
theta_est = zeros(1,length(t));

Sigma_x = zeros(1,length(t));
Sigma_y = zeros(1,length(t));
Sigma_theta = zeros(1,length(t));

range = zeros(length(t),num_landmarks);
bearing = zeros(length(t),num_landmarks);

% FAST-SLAM stuff

% number of particles
M = 200;

% allocate memory to hold particles representing the robot's state
% initialize them all to zero (we know we're at [0 0 0]' when t==0
Y_t_x = zeros(3, M);
Y_t_mu = zeros(2*N, M);         % [[x,y]',[x,y]',[x,y]', ...]
Y_t_Sigma = zeros(2*N, 2*M);

% vector to hold weights
W_t = zeros(1,M);
p0 = 0.1;   % default importance weight

% vector for storing landmark locations
landmark_locations = zeros(2*N, length(t));

% plot the first time
first = 0;
x = 0;
y = 0;
theta = 0;
drawRobot(x,y,theta,landmarks, Y_t_x, first);
first = 1;
pause(0.1)
beam_width = degtorad(90);
% loop through each time step
for i=1:length(t)
%     pause(0.01)
    % Task 1: Implement velocity motion model (Table 5.3)
    v_hat = v_c(i) + randn*sqrt(alpha_1*(v_c(i))^2 + alpha_2*(w_c(i))^2);
    w_hat = w_c(i) + randn*sqrt(alpha_3*(v_c(i))^2 + alpha_4*(w_c(i))^2);
    gamma_hat = randn*sqrt(alpha_5*(v_c(i))^2 + alpha_6*(w_c(i))^2);
    x = x - (v_hat/w_hat)*sin(theta) + (v_hat/w_hat)*sin(theta + w_hat*Ts);
    y = y + (v_hat/w_hat)*cos(theta) - (v_hat/w_hat)*cos(theta + w_hat*Ts);
    theta = theta + w_hat*Ts + gamma_hat*Ts;
    
    % save some data for plotting
    x_true(i) = x;
    y_true(i) = y;
    theta_true(i) = theta;
    
    % Task 2: Simulate range and bearing measurements to landmarks
    ranges = getranges(x, y, landmarks, sigma_r);
    range(i,:) = ranges;
    
    bearings = getbearings(x, y, theta, landmarks, sigma_phi);
    bearing(i,:) = bearings; 
    
    % Task 3: Implement FastSLAM 1.0 algorithm from Table 13.1

    % for each particle...
    for k=1:M
        
        % control input at time t
        u = [v_c(i); w_c(i)];
        
        % line 3, get the previous belief for the kth particle
        x_t_prev = Y_t_x(:,k);
        mu_prev = Y_t_mu(:,k);
        Sigma_prev = Y_t_Sigma(:,2*k-1:2*k);
        
        % line 4, sample pose (propagate each particle through the probabalistic motion model)
        x_t = sample_motion_model(u, x_t_prev, input_noise, Ts);
        Y_t_x(:,k) = x_t;
        
        % step through each landmark
        for j=1:N
            
            % line 6, if feature j hasn't been seen before
            if mu_prev(2*j-1) == 0 && mu_prev(2*j) == 0
                
                % line 7, initialize mean using inverse measurement model
                mu_x = x_t(1) + (ranges(j)*cos(bearings(j) + x_t(3)));
                mu_y = x_t(2) + (ranges(j)*sin(bearings(j) + x_t(3)));
                
                % add to Y_t for the next time around...
                Y_t_mu(2*j-1,k) = mu_x;
                Y_t_mu(2*j,k) = mu_y;
                
                % line 8, calculate Jacobian
                delta_x = mu_x - x_t(1); % m_x - x_x
                delta_y = mu_y - x_t(2);   % m_y - x_y
                delta = [delta_x, delta_y]';
                q = delta'*delta;
                
                H = 1/q * [sqrt(q)*delta_x, sqrt(q)*delta_y;
                    -delta_y, delta_x];
                
                % line 9, initialize covariance
                invH = inv(H);
                Sigma = invH*Q_t*invH';
                % line 10, assign default importance weight
                W_t(k) = p0;
                
                % add to Y_t for next time around...
                Y_t_Sigma(2*j-1:2*j, 2*k-1:2*k) = Sigma;
            else
                
                % get measurement to the current landmark
                range_meas = ranges(j);
                bearing_meas = bearings(j);
                
                % wrap the measurement
                while bearing_meas > pi
                    bearing_meas = bearing_meas - 2 * pi;
                end
                while bearing_meas < -pi
                    bearing_meas = bearing_meas + 2 * pi;
                end
                
                % check to see if the landmark is within the beam_width
                if abs(bearing_meas) > beam_width/2
                    update = 0;
                else
                    update = 1;
                end
                % disp(update)
                if update
                    % line 12, measurement prediction
                    zhat = h(Y_t_mu(2*j-1:2*j,k), x_t);
                    
                    % compute residual
                    residual = [range_meas; bearing_meas] - zhat;
                    
                    % wrap the residual
                    while residual(2) > pi
                        residual(2) = residual(2) - 2 * pi;
                    end
                    while residual(2) < -pi
                        residual(2) = residual(2) + 2 * pi;
                    end
                    
                    % line 13, calculate Jacobian
                    delta_x = Y_t_mu(2*j-1,k) - x_t(1); % m_x - x_x
                    delta_y = Y_t_mu(2*j,k) - x_t(2);   % m_y - x_y
                    delta = [delta_x, delta_y]';
                    q = delta'*delta;
                    
                    % from line 16 of EKF-SLAM
                    H = 1/q * [sqrt(q)*delta_x, sqrt(q)*delta_y;
                        -delta_y, delta_x];
                    
                    % line 14, measurement covariance
                    Q = H * Y_t_Sigma(2*j-1:2*j,2*k-1:2*k) * H' + Q_t;
                    Q = 10*Q;
                    
                    % line 15, calculate Kalman gain
                    K = Y_t_Sigma(2*j-1:2*j,2*k-1:2*k) * H' * inv(Q);
                    
                    % line 16, update the mean
                    Y_t_mu(2*j-1:2*j,k) = Y_t_mu(2*j-1:2*j,k) + K * (residual);   % probably need some wrapping here
                    
                    % line 17, update covariance
                    Y_t_Sigma(2*j-1:2*j, 2*k-1:2*k) = (eye(2) - K * H) * Y_t_Sigma(2*j-1:2*j, 2*k-1:2*k);
                    
                    % line 18, assign importance factor (weight)
                    W_t(k) = W_t(k) + log(mvnpdf(residual, [0,0]', Q));
                    
                else
                    % line 21, leave mean unchanged

                    % line 22, leave covariance unchanged
                end
            end
        end
    end
    
    % we've gone through all the particles, and assigned a weight, now we
    % need to resample...
    
    W_t = exp(W_t - max(W_t));
    % normalize
    W_t = W_t/sum(W_t);
    
    [Y_t_x, Y_t_mu, Y_t_Sigma] = LVS_Y(Y_t_x, Y_t_mu, Y_t_Sigma, W_t, N);
    % END FAST-SLAM 1.0
    
    % reset your weights
    W_t = zeros(1,M);
    
    % update the plot
    drawRobot(x,y,theta,landmarks, Y_t_x, first)
    pause(0.001)
    
    % store some data for later plots
    x_est(i) = mean(Y_t_x(1,:));
    y_est(i) = mean(Y_t_x(2,:));
    theta_est(i) = mean(Y_t_x(3,:));
    
    for z=1:N
        landmark_locations(2*z-1,i) = mean(Y_t_mu(2*z-1,:));
        landmark_locations(2*z,i) = mean(Y_t_mu(2*z,:));
    end
    
end
plot(x_true, y_true, x_est, y_est,'-.')
for i = 1:num_landmarks
    plot(landmark_locations(2*i-1,length(t)), landmark_locations(2*i, length(t)), 'ob')
end

%% plots
figure(2), clf
subplot(3,1,1)
plot(t, x_true, t, x_est,'-.')
title('Robot X-Position')
ylabel('x pos (m)')
legend('truth','estimate')

subplot(3,1,2)
plot(t, y_true, t, y_est,'-.')
title('Robot Y-Position')
ylabel('y pos (m)')
legend('truth','estimate')

subplot(3,1,3)
plot(t,theta_true,t,theta_est,'-.')
title('Robot Heading vs Time')
xlabel('time (s)')
ylabel('heading (rad)')
legend('truth','estimate')

