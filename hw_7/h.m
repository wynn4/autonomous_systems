function zhat = h(landmark_location, robot_state)

delta = [landmark_location(1) - robot_state(1);     
         landmark_location(2) - robot_state(2)];
     
q = delta' * delta;

zhat = [sqrt(q);
        atan2(delta(2), delta(1)) - robot_state(3)];

end

