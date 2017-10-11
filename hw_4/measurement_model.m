function w_t = measurement_model(z, x_t, map, noise)

    % vector to hold the probabilities
    q = zeros(3,1);
    
    % noise terms
    sigma_r = noise(1);
    sigma_phi = noise(2);
    
    % Algorithm from Table 6.4
    for i = 1:length(map)
        % landmark location
        m_x = map(1,i);
        m_y = map(2,i);
        
        % robot location
        x = x_t(1);
        y = x_t(2);
        theta = x_t(3);
        
        % measurements
        r = z(1,i);
        phi = z(2,i);
        
        % compute range and bearing from the particle to the landmark
        r_hat = sqrt((m_x - x)^2 + (m_y -y)^2);
        phi_hat = atan2(m_y - y, m_x - x) - theta;
        
        % compute the probability of getting the current measurement when
        % you're at the current location of the particle
        q(i) = prob(r - r_hat, sigma_r) * prob(phi - phi_hat, sigma_phi);
        
    end
    
    % total weight for the given particle
    w_t = q(1)*q(2)*q(3);
end


function q = prob(dist, std_dev)
    mean = 0;
    q = normpdf(dist, mean, std_dev);
end