function drawRobot(x, y, theta, landmarks, particles, first)
    landmarks_x = landmarks(1,:);
    landmarks_y = landmarks(2,:);
    
    persistent robot_handle
    persistent line_handle
    persistent particles_handle
%     particle_handles = gobjects(length(particles), 1);
    
    if first==0
        figure(1), clf
        
        % initialize plot and draw landmarks
        plot(landmarks_x, landmarks_y, 'or','LineWidth',2)
        hold on
        robot_handle = drawBody(x,y,theta, []);
        line_handle = drawLine(x,y,theta, []);
        particles_handle = scatter(particles(1,:), particles(2,:), 5, 'filled', 'b');
        
        
        title('Mobile Robot Environment')
        xlabel('x pos (m)')
        ylabel('y pos (m)')
        axis([-10, 10, -10, 10]);
        axis equal
        
    % at every other time step, redraw
    else
        drawBody(x,y,theta,robot_handle);
        drawLine(x,y,theta,line_handle);
        particles_handle.XData = particles(1,:);
        particles_handle.YData = particles(2,:);
    end
end
