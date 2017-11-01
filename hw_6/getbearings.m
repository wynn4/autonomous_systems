function bearings = getbearings(x, y, theta, landmarks, sigma_phi)

num_landmarks = length(landmarks);

% allocate
bearings = zeros(1,num_landmarks);

for i = 1:num_landmarks
    bearings(1,i) = atan2(landmarks(2,i) - y, landmarks(1,i) - x) - theta + randn*sigma_phi;  % measurement + noise
end

end