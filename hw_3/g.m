function Chi_bar_t_x_new = g(u, Chi_t_aug_old, dt)

    % inputs
    v_t = u(1,1);
    w_t = u(2,1);
    
    v_i_t = v_t + Chi_t_aug_old(4,:);
    w_i_t = w_t + Chi_t_aug_old(5,:);
    theta_i_t = Chi_t_aug_old(3,:);
    
    x_component = -(v_i_t/w_i_t) * sin(theta_i_t) + (v_i_t/w_i_t) * sin(theta_i_t + w_i_t * dt);
    y_component = (v_i_t/w_i_t) * cos(theta_i_t) - (v_i_t/w_i_t) * cos(theta_i_t + w_i_t * dt);
    theta_component = w_i_t * dt;
    
    Chi_bar_t_x_new = Chi_t_aug_old(1:3,:) + [x_component; y_component; theta_component];
end