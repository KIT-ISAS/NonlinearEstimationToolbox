
function FilterSetExample()
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
    filters.add(filter);
    
    % Use the filter name to get a filter instance
    filters.get('S2KF').setNumSamples(201);
    
    filter = EKF();
    filters.add(filter);
    
    filter = SIRPF('SIR-PF 1000');
    filter.setNumParticles(1000);
    filters.add(filter);
    
    filter = SIRPF('SIR-PF 2000');
    filter.setNumParticles(2000);
    filters.add(filter);
    
    % Initial state estimate
    initialState = Gaussian([1 1 0 0 0]', [10, 10, 1e-1, 1, 1]);
    
    filters.setStates(initialState);
    
    % Just for the heck of it
    for i = 1:filters.getNumFilters()
        % Additionally, you can access each filter by its index
        % Note: the filters are stored in alphabetical order, e.g., the S2KF was added before the EKF.
        filter = filters.get(i);
        
        name  = filter.getName();
        state = filter.getState();
        
        fprintf('* The "%s" estimates the system state as a %s distribution.\n', name, class(state));
        
        if Checks.isClass(state, 'Gaussian')
            [mean, cov] = state.getMeanAndCov();
            fprintf('  Mean and covariance:\n');
            disp([mean cov]);
        elseif Checks.isClass(state, 'DiracMixture')
            fprintf('  Number of mixture components (samples): %d\n', state.getNumComponents());
            [mean, cov] = state.getMeanAndCov();
            fprintf('  Mean and covariance:\n');
            disp([mean cov]);
        else
            fprintf('  Unexpected distribution\n');
        end
    end
    
    % Perform a prediction step
    filters.predict(sysModel);
    
    % Show the predicted state estimates
    printPointEstimates(filters);
    
    % Assume we receive the measurement
    measurement = [3 pi/5]';
    
    % Perform a measurement update
    filters.update(measModel, measurement)
    
    % Show the filtered state estimates
    printPointEstimates(filters);
end

function printPointEstimates(filters)
    numFilters = filters.getNumFilters();
    names      = filters.getNames();
    
    [stateMeans, stateCovs] = filters.getPointEstimates();
    
    for i = 1:numFilters
        fprintf('\nFilter: %s\n', names{i});
        fprintf('State mean and covariance:\n');
        disp([stateMeans(:, i) stateCovs(:, :, i)]);
    end
end
