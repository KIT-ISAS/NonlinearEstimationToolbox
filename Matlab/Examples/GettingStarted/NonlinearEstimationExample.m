
function NonlinearEstimationExample()
    % Instantiate system model
    sysModel = TargetSysModel();
    
    % Instantiate measurement Model
    measModel = PolarMeasModelNaive();
%     measModel = PolarMeasModel();
%     measModel = PolarMeasLikelihood();
    
    % The estimator
    filter = UKF();
%     filter = GHKF();
%     filter = EKF();
%     filter = SIRPF();
%     filter = GPF();
%     filter.setNumParticles(10^5);
    
    % Initial state estimate
    initialState = Gaussian([1 1 0 0 0]', [10, 10, 1e-1, 1, 1e-1]);
    
    filter.setState(initialState);
    
    % Perform a prediction step
    filter.predict(sysModel);
    
    % Show the predicted state estimate
    fprintf('Predicted state estimate:\n\n');
    printStateMeanAndCov(filter);
    
    % Assume we receive the measurement
    measurement = [3 pi/5]';
    
    % Perform a measurement update
    filter.update(measModel, measurement);
    
    % Show the updated state estimate
    fprintf('Updated state estimate:\n\n');
    printStateMeanAndCov(filter);
end

function printStateMeanAndCov(filter)
    [mean, cov] = filter.getStateMeanAndCov();
    
    fprintf('State mean:\n');
    disp(mean);
    
    fprintf('State covariance matrix:\n');
    disp(cov);
    
    fprintf('\n');
end
