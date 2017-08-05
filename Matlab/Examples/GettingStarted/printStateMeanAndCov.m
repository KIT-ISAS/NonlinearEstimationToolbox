
function printStateMeanAndCov(filter)
    [mean, cov] = filter.getStateMeanAndCov();
    
    fprintf('State mean:\n');
    disp(mean);
    
    fprintf('State covariance:\n');
    disp(cov);
    
    fprintf('\n');
end
