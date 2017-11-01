function ranges = getranges(x, y, landmarks, sigma_r)

num_landmarks = length(landmarks);

% allocate
ranges = zeros(1, num_landmarks);

for i = 1:num_landmarks
    ranges(1,i) = norm([x; y] - landmarks(:,i)) + randn*sigma_r;   % measurement + noise
end

end