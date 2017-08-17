
function StandardKFExample()
    %%%%% Scenario
    % 2D system state
    %
    %   x(k) = [p(k) s(k)]'
    %
    % containing scalar position p(k) and scalar speed s(k)
    
    %%%%% System Model
    % Constant velocity system model
    %
    %   x(k) = A * x(k-1) + B * w
    
    % Delta t (step size)
    T = 0.01;
    
    % System matrix A
    sysMatrix      = [1 T
                      0 1];
    
    % System noise matrix B
    sysNoiseMatrix = [T
                      1];
    
    % Time-invariant, zero-mean Gaussian white noise w
    sysNoise = Gaussian(0, 0.1^2);
    
    % Create system model instance and set its parameters
    sysModel = LinearSystemModel();
    sysModel.setSystemMatrix(sysMatrix);
    sysModel.setSystemNoiseMatrix(sysNoiseMatrix);
    sysModel.setNoise(sysNoise);
    
    % A few notes on the linear system model
    %
    %  * If no system matrix is provided, it is assumed to be the identity
    %    matrix of appropriate dimensions.
    %
    %  * If no system noise matrix is provided, it is assumed that
    %
    %       dim(w(k)) == dim(x(k))
    %
    %    and the system noise matrix will be set to the identity of
    %    appropriate dimensions.
    %
    % * Hence, if both matrices are not set, the LinearSystemModel acts
    %   like a random walk system model.
    %
    % * It is possible to pass the system matrix and system noise matrix
    %   directly to the class constructor of the LinearSystemModel to avoid
    %   the additional set methods.
    %
    % * The system noise does not have to be distributed according to
    %   a Gaussian distribution. One can pass here any other distribution,
    %   as the Kalman filter internally only works with the distribution's
    %   mean and covariance. However, in such a case the KF isn't anymore
    %   the optimal MMSE estimator (only a linear MMSE estimator).
    
    %%%%% Measurement Model
    % Linear measurement model
    %   y(k) = H * x(k) + v
    
    % Measurement matrix H
    H = [1 0];
    
    % Time-invariant, zero-mean Gaussian white noise v
    measNoise = Gaussian(0, 0.05^2);
    
    % Create measurement model instance and set its parameters
    measModel = LinearMeasurementModel();
    measModel.setMeasurementMatrix(H);
    measModel.setNoise(measNoise);
    
    % A few notes on the linear measurement model
    %
    % * If no measurement matrix is provided, it is assumed to be the
    %   identity matrix of appropriate dimensions.
    %
    % * It is possible to pass the measurement matrix directly to
    %   the class constructor of LinearMeasurementModel to avoid the
    %   additional set method.
    %
    % * The measurement noise does not have to be distributed according to
    %   a Gaussian distribution. One can pass here any other distribution,
    %   as the Kalman filter internally only works with the distribution's
    %   mean and covariance. However, in such a case the KF isn't anymore
    %   the optimal MMSE estimator (only a linear MMSE estimator).
    
    %%%%% The estimator
    % As both models are linear, state prediction and measurement update
    % can be fully calculated in closed-form by a Kalman filter.
    % Consequently, all implemented Kalman filters handle these models
    % in an analytic way, e.g., without calculating derivates or using samples.
    % Hence, we simply choose to use the extended Kalman filter (EKF)
    % implementation in this example.
    filter = EKF();
    
    % Set initial state estimate
    %   E[x(0)]    = [0 0]'
    %   Cov[x(0)]  = diag([1 1])
    
    initialState = Gaussian(zeros(2, 1), eye(2));
    
    filter.setState(initialState);
    
    % A few notes on on setting the system state
    %
    %  * The initial state estimate can be of any distribution as the Kalman filter
    %    internally only works with the distribution's mean and covariance matrix.
    %    For example, a Gaussian mixture will be reduced to its mean and
    %    covariance matrix, and thus, will lose any existing multi-modality.
    %
    %  * The system state can be set to any value to any time by calling
    %    the setState() class method (e.g., when the system state estimate
    %    needs to be re-initialized due to a track loss).
    
    % Perform a prediction step
    filter.predict(sysModel);
    
    % Print predicted state mean and state covariance matrix by getting its
    % Gaussian state estimate:
    predState = filter.getState();
    
    [mean, cov] = predState.getMeanAndCov();
    
    fprintf('Predicted state mean:\n');
    disp(mean);
    
    fprintf('Predicted state covariance:\n');
    disp(cov);
    
    fprintf('\n');
    
    % Assume we receive the measurement
    measurement = 2.139;
    
    % Perform a measurement update
    filter.update(measModel, measurement);
    
    % Print the updated state estimate. However, instead of using a combination
    % of getState() and getMeanAndCov() as above, we now use the handy
    % Filter's getStateMeanAndCov() method:
    [mean, cov] = filter.getStateMeanAndCov();
    
    fprintf('Updated state mean:\n');
    disp(mean);
    
    fprintf('Updated state covariance:\n');
    disp(cov);
    
    fprintf('\n');
end
