function chi_t_new = sample_motion_model(u, chi_t_old, noise, dt)

    % noise params
    alpha_1 = noise(1);
    alpha_2 = noise(2);
    alpha_3 = noise(3);
    alpha_4 = noise(4);
    alpha_5 = noise(5);
    alpha_6 = noise(6);
    
    
    % get the input velocities
    v_t = u(1);
    w_t = u(2);
    
    % previoius heading
    theta_t = chi_t_old(3);
    
    % noise components
    x_noise = randn*sqrt(alpha_1*(v_t)^2 + alpha_2*(w_t)^2);
    y_noise = randn*sqrt(alpha_3*(v_t)^2 + alpha_4*(w_t)^2);
    gamma_hat = randn*sqrt(alpha_5*(v_t)^2 + alpha_6*(w_t)^2);
    
    % calculate how much the state changed in x, y, and theta using the
    % motion model
    x_component = -(v_t/w_t) * sin(theta_t) + (v_t/w_t) * sin(theta_t + w_t * dt) + x_noise;
    y_component = (v_t/w_t) * cos(theta_t) - (v_t/w_t) * cos(theta_t + w_t * dt) + y_noise;
    theta_component = w_t * dt + gamma_hat;
    
    % new state = old state + change in state
    chi_t_new = chi_t_old + [x_component; y_component; theta_component];
end
