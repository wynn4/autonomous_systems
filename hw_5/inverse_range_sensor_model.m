function l = inverse_range_sensor_model(m, x_t, z_t, alpha, beta, z_max, theta_k)

    l0 = log(0.5/(1-0.5));
    locc = log(0.7/(1-0.7));
    lfree = log(0.3/(1-0.3));
    
    
    % get the x and y coordinates of the center of the grid cell
    xi = m(1);
    yi = m(2);
    
    % get the state of the robot
    x = x_t(1);
    y = x_t(2);
    theta = x_t(3);
    
    % compute the range
    r = sqrt((xi - x)^2 + (yi - y)^2);
    phi = atan2(yi - y, xi - x) - theta;
    % he gives us theta_k so we don't need this
    % k = argmin(abs(phi - theta_sens));
    
    if r > min(z_max, z_t + alpha/2) || abs(phi - theta_k) > beta/2
        l = l0;
    
    
    elseif z_t < z_max && abs(r - z_t) < alpha/2
        l = locc;
    
    
    elseif r <= z_t
        l = lfree;
    
    else
        l = -999;   % this case shouldn't happen but sometimes it does?
    end
    

end