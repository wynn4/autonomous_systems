function drawRobot(x, y, theta, landmarks, first)
    landmarks_x = landmarks(1,:);
    landmarks_y = landmarks(2,:);
    
    persistent robot_handle
    persistent line_handle
    
    if first==0
        figure(1), clf
        
        % initialize plot and draw landmarks
        plot(landmarks_x, landmarks_y, 'or','LineWidth',2)
        hold on
        robot_handle = drawBody(x,y,theta, []);
        line_handle = drawLine(x,y,theta, []);
        xlabel('x pos (m)')
        ylabel('y pos (m)')
        axis([-10, 10, -10, 10]);
        axis equal
        
    % at every other time step, redraw
    else
        drawBody(x,y,theta,robot_handle);
        drawLine(x,y,theta,line_handle);
    end
end
