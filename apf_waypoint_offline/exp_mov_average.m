function x_filtered = exp_mov_average(x, alpha)

    if ~isequal(size(x), [3, 1])
        error('L''input x deve essere un vettore colonna 3x1.');
    end
    
    persistent x_filtered_prev
    
    if isempty(x_filtered_prev)
        x_filtered_prev = [0 0 0]';
    end
    
    x_filtered = (1 - alpha) .* x_filtered_prev + alpha .* x;
    
    x_filtered_prev = x_filtered;
end