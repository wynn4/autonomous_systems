function [u_hat, value] = policy_POMDP(Y, b)

values = (Y(:,2) - Y(:,3)) * b + Y(:,3);

% get the index of the maximum value
[value, idx] = max(values);

% return the u associated with that index and the value at that index
u_hat = Y(idx, 1);

end