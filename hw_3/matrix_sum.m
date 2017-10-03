function summation = matrix_sum(weights, matrix)
    
    % get the matrix dimension
    dim = size(matrix);
    rows = dim(1);
    cols = dim(2);
    
    % initialize sum
    summation = zeros(rows, 1);
    
    for i = 1:rows
        row = matrix(i,:);
        summation(i) = sum(weights .* row);
    end
    
%     disp(summation)
end