function handle = drawLine(x, y, theta, handle)
    w = 1; % square robot width
%     robot_pts = [x+ w/2, x + w/2, x + -w/2, x + -w/2;
%                  y + w/2, y + -w/2, y + -w/2, y + w/2];
    line_pts = [0, w/2;
                        0, 0];
    pos = [x; y];
                    
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % first rotate about the origin
    line_pts = R*line_pts;
    % then move it out to the desired point
    line_pts = line_pts + pos;
    X = line_pts(1,:);
    Y = line_pts(2,:);
    
    if isempty(handle)
        handle = fill(X,Y,'k','LineWidth',2);
    else
        set(handle,'XData',X,'YData',Y);
    end
    
    

end