function Y_prime_pruned = Pruninator(Y_prime)

% throw out any duplicates from the start
Y_prime = unique(Y_prime, 'rows');

% chop up [0, 1] into 1000 tiny pieces
x = 0:0.001:1;

% find the height of each line at every x value
y = (Y_prime(:,3) - Y_prime(:,2)) * x + Y_prime(:,2);

% get the index of the max of each row
[~, idx] = max(y,[],1);

% get only the unique indexes
pruned = unique(idx);

% now mask off Y_prime using the pruned indexes
Y_prime_pruned = Y_prime(pruned,:);



end