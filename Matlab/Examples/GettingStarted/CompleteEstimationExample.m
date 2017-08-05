
function CompleteEstimationExample()
    % Instantiate system model
    sysModel = TargetSysModelB();
    
    % Set Delta T (discrete time step size)
    sysModel.deltaT = 0.01;
    
    % Set time-invariant, zero-mean Gaussian system noise
    sysNoise = Gaussian(zeros(5, 1), [1e-3 1e-3 1e-5 1e-3 1e-5]);
    
    sysModel.setNoise(sysNoise);
    
    % Instantiate measurement Model
    measModel = PolarMeasModelB();
    
    % Set time-invariant, zero-mean Gaussian measurement noise
    measNoise = Gaussian(zeros(2, 1), [1e-2 1e-4]);
    
    measModel.setNoise(measNoise);
    
    % Setup the filters
    filters = FilterSet();
    
    filter = S2KF();
    filter.setNumSamples(201);
    filter.setColor({ 'Color', [0 0.5 0] });
    filters.add(filter);
    
    filter = EKF();
    filter.setColor({ 'Color', 'r' });
    filters.add(filter);
    
    filter = SIRPF('SIR-PF 1000');
    filter.setNumParticles(1000);
    filter.setColor({ 'Color', 'm' });
    filters.add(filter);
    
    filter = SIRPF('SIR-PF 2000');
    filter.setNumParticles(2000);
    filter.setColor({ 'Color', 'c' });
    filters.add(filter);
    
    numFilters = filters.getNumFilters();
    
    % Initial state estimate
    initialState = Gaussian([1 1 0 0 0]', [10, 10, 1e-1, 1, 1]);
    
    filters.setStates(initialState);
    
    % Simulate system state trajectory and noisy measurements
    numTimeSteps = 200;
    
    sysStates          = nan(5, numTimeSteps);
    measurements       = nan(2, numTimeSteps);
    stateMeans         = nan(5, numFilters, numTimeSteps);
    stateCovs          = nan(5, 5, numFilters, numTimeSteps);
    runtimesUpdate     = nan(numFilters, numTimeSteps);
    runtimesPrediction = nan(numFilters, numTimeSteps);
    
    sysState = initialState.drawRndSamples(1);
    
    for k = 1:numTimeSteps
        % Simulate measurement for time step k
        measurement = measModel.simulate(sysState);
        
        % Perform a measurement update
        runtimesUpdate(:, k) = filters.update(measModel, measurement);
        
        sysStates(:, k)    = sysState;
        measurements(:, k) = measurement;
        
        [stateMeans(:, :, k), stateCovs(:, :, :, k)] = filters.getStateMeansAndCovs();
        
        % Simulate next system state
        sysState = sysModel.simulate(sysState);
        
        % Perform a prediction step
        runtimesPrediction(:, k) = filters.predict(sysModel);
    end
    
    % Plot true system state and state estimates
    figure();
    hold on;
    axis equal;
    grid on;
    xlabel('x');
    ylabel('y');
    
    plot(sysStates(1, :), sysStates(2, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'System State');
    plot(sysStates(1, 1), sysStates(2, 1), 'bo', 'LineWidth', 3, 'DisplayName', 'Start');
    
    for i = 1:numFilters
        filter = filters.get(i);
        color  = filter.getColor();
        name   = filter.getName();
        
        means = squeeze(stateMeans(:, i, :));
        
        plot(means(1, :), means(2, :), '-', 'LineWidth', 1.5, color{:}, 'DisplayName', name);
        plot(means(1, 1), means(2, 1), 'o', 'LineWidth', 3, color{:}, 'DisplayName', 'Start');
    end
    
    legend show;
    
    % Plot prediction and measurement update runtimes
    figure();
    hold on;
    xlabel('Time step');
    ylabel('Runtime in ms');
    
    for i = 1:numFilters
        filter = filters.get(i);
        color  = filter.getColor();
        name   = filter.getName();
        
        r = runtimesUpdate(i, :) * 1000;
        plot(1:numTimeSteps, r, '-', 'LineWidth', 1.5, color{:}, 'DisplayName', sprintf('Update %s', name));
        
        r = runtimesPrediction(i, :) * 1000;
        plot(1:numTimeSteps, r, '--', 'LineWidth', 1.5, color{:}, 'DisplayName', sprintf('Prediction %s', name));
    end
    
    legend show;
end
