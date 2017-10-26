function val = sat(num)
    max = 1;
    min = 0;
    
    if num >= max
        val = max;
    elseif num <= min
        val = min;
    else
        val = num;
    end
end