function handle = drawBody(x, y, theta, handle)
    w = 1; % square robot width
    robot_pts = [w/2, w/2, -w/2, -w/2;
                 w/2, -w/2, -w/2, w/2];
%     heading_line_pts = [x, x;
%                         y + w/2, y];

    pos = [x; y];
                    
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % first rotate about the origin
    robot_pts = R*robot_pts;
    % then move it out to the desired point
    robot_pts = robot_pts + pos;
    X = robot_pts(1,:);
    Y = robot_pts(2,:);
    
    if isempty(handle)
        handle = fill(X,Y,'g');
    else
        set(handle,'XData',X,'YData',Y);
    end
    
    

end