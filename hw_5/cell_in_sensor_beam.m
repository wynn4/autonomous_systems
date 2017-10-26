function status = cell_in_sensor_beam(m, x_t)
    
    % cell location
    mx = m(1);
    my = m(2);
    
    % robot location
    x = x_t(1);
    y = x_t(2);
    theta = x_t(3);
    
    % basically just check if the cell is forward of the nose of the robot
    %
    % stuff here to do the check
    
    % for now just assume every cell is visible to the lasers
    status = 1;
    return
end