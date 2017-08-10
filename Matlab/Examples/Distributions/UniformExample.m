
function UniformExample()
    %%%%%%%%%%%%%%%%%%%%%%%
    % Configure
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Configure a scalar uniform distribution over the interval [-5, 3]:
    u1 = Uniform(-5, 3);
    
    % Configure a 2D axis-aligned uniform distribution over the region [-2, 2] x [3, 4]:
    u2 = Uniform([-2 3], [2, 4]);
    
    % An already initialized Uniform can be changed by using its set() method:
    u1.set(0, 1);
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Get Information
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Dimension of the distribution:
    dim = u2.getDim();
    
    % Get mean, covariance matrix and the lower Cholesky decomposition of
    % the distribution's covariance matrix:
    [mean, cov, covSqrt] = u2.getMeanAndCov();
    
    % Get the support of the uniform distribution:
    [a, b] = u2.getInterval();
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Draw Random Samples
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Draw eight random samples from u2:
    samples = u2.drawRndSamples(8);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Evaluate Logarithmic Probability Density Function (PDF)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get log PDF values of u2 for the points [1, 3.5]' and [-1.5 4.5]':
    logValues = u2.logPdf([1.0 -1.5
                           3.5  4.5]);
    
    % Or plot entire PDF of u2:
    t      = -5:0.1:5;
    [x, y] = meshgrid(t);
    pos    = [x(:)'
              y(:)'];
    
    values = exp(u2.logPdf(pos));
    
    surf(x, y, reshape(values, [length(t) length(t)]));
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Copy
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Here, u is only a reference to u1 (no data is copied):
    u = u1;
    
    u.set([-1 -1], [1 1]);
    
    % Hence, the above set() call also changes u1:
    [mean, cov] = u1.getMeanAndCov();
    
    % Use the distribution's copy() method for a real copy:
    u = u1.copy();
    
    u.set(5, 10);
    
    % Now, u1 remains unchanged:
    [mean, cov] = u1.getMeanAndCov();
end
