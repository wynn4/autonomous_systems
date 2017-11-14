function [Y_t_x_new, Y_t_mu_new, Y_t_Sigma_new] = LVS_Y(Y_t_x, Y_t_mu, Y_t_Sigma, W_t, N)

    % Algorithm Low Variance Sampler from Table 4.4 (p. 110)
    
    % number of samples
    M = length(Y_t_x);
    
    % initialize Y_t
    Y_t_x_new = zeros(3, M);
    Y_t_mu_new = zeros(2*N, M);
    Y_t_Sigma_new = zeros(2*N, 2*M);
    
    a = 0;
    b = 1/M;
    
    % choose a random number between 0 and 1/M
    r = a + (b-a).*rand(1,1);
    
    c = W_t(1);
    
    i = 1;
    
    for m = 1:M
        U = r + (m - 1)*(1/M);
        while U > c
            i = i + 1;
            c = c + W_t(i);
        end
        
        Y_t_x_new(:,m) = Y_t_x(:,i);
        Y_t_mu_new(:,m) = Y_t_mu(:,i);
        Y_t_Sigma_new(:,2*m-1:2*m) = Y_t_Sigma(:,2*i-1:2*i);
    end
    
    
end