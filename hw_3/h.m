function z = h(m, Chi_bar_t_x, Chi_t_z)
    m_x = m(1);
    m_y = m(2);
    
    % get the dimension of the matrix coming in
    dim = size(Chi_bar_t_x);
    cols = dim(2);
    
    % initialize z
    z = zeros(2,cols);
    
    for i = 1:cols
        range = sqrt((m_x - Chi_bar_t_x(1,i))^2 + (m_y - Chi_bar_t_x(2,i))^2) + Chi_t_z(1,i);
        bearing = atan2(m_y - Chi_bar_t_x(2,i), m_x - Chi_bar_t_x(1,i)) - Chi_bar_t_x(3,i) + Chi_t_z(2,i);
        z(1,i) = range;
        z(2,i) = bearing;
    end
end