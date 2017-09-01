
function FilterSetExample()
    % Instantiate system model
    sysModel = TargetSysModel();
    
    % Instantiate measurement Model
    measModel = PolarMeasModel();
    
    % Setup the filters
    filters = FilterSet();
    
    filter = GHKF('GHKF');
    filters.add(filter);
    
    % Use the filter name to get a filter instance
    filters.get('GHKF').setNumQuadraturePoints(4);
    
    filter = EKF();
    filters.add(filter);
    
    filter = EKF('Iterative EKF');
    filter.setMaxNumIterations(10);
    filters.add(filter);
    
    filter = SIRPF();
    filter.setNumParticles(10^5);
    filters.add(filter);
    
    % Initial state estimate
    initialState = Gaussian([1 1 0 0 0]', [10, 10, 1e-1, 1, 1e-1]);
    
    filters.setStates(initialState);
    
    % Just for the heck of it:
    for i = 1:filters.getNumFilters()
        % Additionally, you can access each filter by its index
        % Note: the filters are stored in alphabetical order, e.g.,
        % the GHKF was added before the EKF.
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
    fprintf('\n\nPredicted state estimates:\n');
    printStateMeansAndCovs(filters);
    
    % Assume we receive the measurement
    measurement = [3 pi/5]';
    
    % Perform a measurement update
    filters.update(measModel, measurement)
    
    % Show the updated state estimates
    fprintf('\n\nUpdated state estimates:\n');
    printStateMeansAndCovs(filters);
end

function printStateMeansAndCovs(filters)
    numFilters = filters.getNumFilters();
    names      = filters.getNames();
    
    [stateMeans, stateCovs] = filters.getStatesMeanAndCov();
    
    for i = 1:numFilters
        fprintf('\nFilter: %s\n', names{i});
        fprintf('State mean and covariance:\n');
        disp([stateMeans(:, i) stateCovs(:, :, i)]);
    end
end
