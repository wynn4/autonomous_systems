clc
clear
close all

% get the map
[map, walls, obs1, obs2, obs3, goal] = getMap();

[Mm,Nm] = size(map);

% discount factor
gamma = 1;

% cost params
wall_cost = -100;
obstacle_cost = -5000;
goal_cost = 100000;
nominal_cost = -2;

% memory allocation
V_prev = zeros(Mm);
V_new = ones(Mm);

% implement MDP_discrete_value_iteration algorithm from Table 14.1

% Allocate space to store the values
V_hat = zeros(Mm);
policy = zeros(Mm);

% Go through and assign initial values to all the cells
for i=1:Mm
    for j = 1:Nm
        if walls(i,j) == 1
            V_hat(i,j) = wall_cost;
        elseif obs1(i,j) == 1
            V_hat(i,j) = obstacle_cost;
        elseif obs2(i,j) == 1
            V_hat(i,j) = obstacle_cost;
        elseif obs3(i,j) == 1
            V_hat(i,j) = obstacle_cost;
        elseif goal(i,j) == 1
            V_hat(i,j) = goal_cost;
        else
            V_hat(i,j) = 0;
        end
    end
end

% done now go through and iterate the value function
count = 0;
V_diff = V_new - V_prev;
while norm(V_diff) > 10
    V_prev = V_hat;
    for i = 2:Mm-1
        for j = 2:Nm-1
            
            % don't update value of obstacles, walls, or goal
            if obs1(i,j) || obs2(i,j) || obs3(i,j)
                continue;
            end
            
            if walls(i,j)
                continue;
            end
            
            if goal(i,j)
                continue;
            end
            
            
            % grab the 3 by 3 matrix around the i,j location
            mat = V_hat(i-1:i+1,j-1:j+1);
            
            [new_value, u] = getValue(mat, nominal_cost, gamma);
            V_hat(i,j) = new_value;
            policy(i,j) = u;
            
        end
    end
    V_new = V_hat;
    V_diff = V_new - V_prev;
    
    % see progress
    count = count + 1;
    disp(norm(V_diff))
end

policy = policy(3:Mm-2,3:Nm-2);

%% plots
figure(1), clf
surf(V_hat)
colormap('jet')
colorbar;
caxis([99500, 100000])
axis 'equal'
view([-90 90])

figure(2), clf
yes_plot = 1;
for i = 1:Mm -4
    for j = 1:Nm -4
        if policy(i,j) == 1
            angle = 0 + degtorad(90);
        elseif policy(i,j) == 2
            angle = degtorad(270) + degtorad(90);
        elseif policy(i,j) == 3
            angle = degtorad(180) + degtorad(90);
        elseif policy(i,j) == 4
            angle = degtorad(90) + degtorad(90);
        else
            yes_plot = 0;
        end
        
        if yes_plot
            x0 = i;
            y0 = j;
            length = 1;
            
            % a better way to do this would be to use quiver() probably
            draw_arrow(x0,y0,length,angle)
            hold on
        end
        yes_plot = 1;
    end
end

% initial robot position
x = 28;
y = 20;

value = 0;
figure(2)
while value < goal_cost

    if policy(x,y) == 1
        new_y = y;
        new_x = x -1;
        value = V_hat(new_y,new_x);
    elseif policy(x,y) == 2
        new_y = y + 1;
        new_x = x;
        value = V_hat(new_y,new_x);
    elseif policy(x,y) == 3
        new_y = y;
        new_x = x + 1;
        value = V_hat(new_y,new_x);
    elseif policy(x,y) == 4
        new_y = y -1;
        new_x = x;
        value = V_hat(new_y,new_x);
    else
        % you've reached your goal (hopefully)
        value = goal_cost;
    end
    
    xx = [x, new_x];
    yy = [y, new_y];
    plot(xx,yy,'-r')
    hold on
    pause(0.01)
    
    y = new_y;
    x = new_x;
end
figure(2)
xlabel('X Pos (m)')
ylabel('Y Pos (m)')
title('Policy and Robot Path to Goal')

