function Chi_bar_t = LVS(Chi_t, W_t)

    % Algorithm Low Variance Sampler from Table 4.4 (p. 110)
    
    % number of samples
    M = length(Chi_t);
    
    % initialize Chi_bar_t
    Chi_bar_t = zeros(3, M);
    
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
        
        Chi_bar_t(:,m) = Chi_t(:,i);
    end
    
    
end