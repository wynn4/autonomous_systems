function Chi_bar_t_x = g(u, Chi_t_aug, dt)
    
    % get the dimension of the matrix coming in
    dim = size(Chi_t_aug);
    cols = dim(2);
    
    % initialize Chi_bar_t_x_new
    Chi_bar_t_x = zeros(3,cols);
    
    for i = 1:cols
        v_t = u(1,1) + Chi_t_aug(4,i);
        w_t = u(2,1) + Chi_t_aug(5,i);
        theta_t = Chi_t_aug(3,i);
        
        x_component = -(v_t/w_t) * sin(theta_t) + (v_t/w_t) * sin(theta_t + w_t * dt);
        y_component = (v_t/w_t) * cos(theta_t) - (v_t/w_t) * cos(theta_t + w_t * dt);
        theta_component = w_t * dt;
        
        Chi_bar_t_x(:,i) = Chi_t_aug(1:3,i) + [x_component; y_component; theta_component];
    end
    
end