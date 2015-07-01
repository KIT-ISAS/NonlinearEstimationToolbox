
function printPointEstimate(filter)
    [mean, cov] = filter.getPointEstimate();
    
    fprintf('State mean:\n');
    disp(mean);
    
    fprintf('State covariance:\n');
    disp(cov);
    
    fprintf('\n');
end
