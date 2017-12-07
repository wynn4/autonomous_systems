clc
clear
close all

tic;
% POMDP Algorithm From Table 15.1

% time horizon
T = 20;
gamma = 1;

% prob of meas z given x
p_z_gvn_x = [0.7, 0.3;
             0.3, 0.7];

% prob of state x given u and previous state
p_x_gvn_u_x = zeros(2,3,2);
p_x_gvn_u_x(:,:,1) = [0, 0, 0.2;
                      0, 0, 0.8];
p_x_gvn_u_x(:,:,2) = [0, 0, 0.8;
                      0, 0, 0.2];
                  
% reward
r = [-100, 100, -1;
     100, -50, -1];
 
v_prime = zeros(1,2);

Y = [0, 0, 0];  % [u, x1, x2]

for tau = 1:T
    
    Y_prime = [];
    [KK,~] = size(Y);
    
    % for however many lines we've got...
    for k = 1:KK
        
        % for all control actions u
        for u = 1:3
            
            % for all measurements z
            for z = 1:2
                
                % for j = 1:Num_states (Num_states is 2 in our case)
                for j = 1:2
                    
                    % value function
                    v_x1 = Y(k,2) * p_z_gvn_x(z,1) * p_x_gvn_u_x(j,u,1);
                    v_x2 = Y(k,3) * p_z_gvn_x(z,2) * p_x_gvn_u_x(j,u,2);
                    
                    v_uzj(k,u,z,j) = v_x1 + v_x2;
                    
                end
                
            end
            
        end
    end
    
    % for all control actions u
    for u = 1:3
        
        % for all K(1),...,k(M) (just do this as a double for-loop)
        for  k1 = 1:KK
            for k2 = 1:KK
                
                % for s = 1:Num_states (Num_states is 2 in our case)
                for j = 1:2
                    
                    v_prime(j) = gamma * (r(j,u) + v_uzj(k1, u, 1, j) + v_uzj(k2, u, 2, j));
                    
                end
                
                % add u, v_prime to Y_prime
                Y_prime = [Y_prime; u, v_prime];
                
            end
        end
        
        
    end
    
    %prune Y_prime
    Y_pruned = Pruninator(Y_prime);
    Y = Y_pruned;
    
end

toc;
% plots
figure(1), clf
hold on
for i = 1:length(Y)
    plot(0:1, fliplr(Y(i, 2:3)))
end






