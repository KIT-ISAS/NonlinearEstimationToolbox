
function NonlinearEstimationExample()
    % Instantiate system model
    sysModel = TargetSysModelA();
%     sysModel = TargetSysModelB();
    
    % Set Delta T (discrete time step size)
    sysModel.deltaT = 0.01;
    
    % Set time-invariant, zero-mean Gaussian system noise
    sysNoise = Gaussian(zeros(5, 1), [1e-3 1e-3 1e-5 1e-3 1e-5]);
    
    sysModel.setNoise(sysNoise);
    
    % Instantiate measurement Model
    measModel = PolarMeasModelA();
%     measModel = PolarMeasModelB();
%     measModel = PolarMeasLikelihood();
    
    % Set time-invariant, zero-mean Gaussian measurement noise
    measNoise = Gaussian(zeros(2, 1), [1e-2 1e-4]);
    
    measModel.setNoise(measNoise);
    
    % The estimator
    filter = UKF();
%     filter = S2KF();
%     filter = EKF();
%     filter = SIRPF();
%     filter = PGF();
    
    % Initial state estimate
    initialState = Gaussian([1 1 0 0 0]', [10, 10, 1e-1, 1, 1]);
    
    filter.setState(initialState);
    
    % Perform a prediction step
    filter.predict(sysModel);
    
    % Show the predicted state estimate
    printStateMeanAndCov(filter);
    
    % Assume we receive the measurement
    measurement = [3 pi/5]';
    
    % Perform a measurement update
    filter.update(measModel, measurement);
    
    % Show the filtered state estimate
    printStateMeanAndCov(filter);
end
