% Homework 5 Occupancy grid mapping
% Jesse Wynn
% October 25, 2017

%% Occupancy Grid Mapping Implementation
clc
clear
close all

% load in the state and measurement data
load('state_meas_data.mat')

% sensor params
alpha = 1;              % meters
beta = degtorad(2);     % radians
z_max = 150;            % meters

% other params
grid_res = 101; % 101 x 101 grid map
grid_size = 1;  % meters

% probability of occupied (eq 9.7)
l0 = log(0.5/(1-0.5));
locc = log(0.7/(1-0.7));
lfree = log(0.3/(1-0.3));

% initialize l to be l0
l = l0 * ones(grid_res);

% initialize the map with equal probability of being free or being occupied
map = 0.5 * ones(grid_res);

% implement Occupancy Grid Mapping algorithm from Table 9.1 and Table 9.2
for i = 1:length(X)
    for m = 1:grid_res
        for n = 1:grid_res
            % get the center location of the cell we're currently on
            % m_cent = [m - grid_size/2, n - grid_size/2];
            m_cent = [m,n]; % this works way better than the line above...don't know why
            if cell_in_sensor_beam(m_cent, X(:,i))
                for p = 1:length(thk)
                    inv_r_s_m = inverse_range_sensor_model(m_cent, X(:,i), z(1,p,i), alpha, beta, z_max, thk(p), l0, locc, lfree);
                    if inv_r_s_m ~= -999    % (-999 is the invalid case)
                        l(m,n) = l(m,n) + inv_r_s_m - l0;
                    end
                end
                
                % convert back to probability land
                prob = exp(l(m,n));
                prob = prob/(1 + prob);
                map(m,n) = prob;
            else
                l(m,n) = l(m,n);    % right now we never go in here because cell_in_sensor_beam() always returns true.
            end
        end
        
    end
    % monitor progress
    % disp(i)
end


%% Plots

figure(1), clf
% plot using surf
surf(map','LineStyle', 'none');
xlabel('X (m)')
ylabel('Y (m)')
title('Robot Environment')
colorbar;
colormap(flipud(gray));
view(0,90)
axis([1 101 1 101])
axis equal
grid off
