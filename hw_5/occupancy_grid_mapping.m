% Homework 5 Occupancy grid mapping
% Jesse Wynn
% October 25, 2017

clc
clear
close all

% load in the state and measurement data
load('state_meas_data.mat')

% sensor params
alpha = 1;              % meters
beta = degtorad(5);     % radians
z_max = 150;            % meters

% other params
grid_res = 100; % 100 x 100 grid map
grid_size = 1;  % meters

% probability of occupied (eq 9.7)
l0 = log(0.6/(1-0.6));

% allocate memory to store map data
m = zeros(grid_res);

% implement Occupancy Grid Mapping algorithm from Table 9.1 and Table 9.2
for i = 1:length(X)
    for m = 1:grid_res
        for n = 1:grid_res
            % get the center location of the cell we're currently on
            m_cent = [m - grid_size/2, n - grid_size/2];
            if cell_in_sensor_beam(m_cent, X)
                for p = 1:length(thk)
                    l = l + inverse_range_sensor_model(m, x_t, z_t, alpha, beta, z_max, theta_k) - l0;
                end
            else
            end
        end
        
    end
end