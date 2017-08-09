
function GaussianExample()
    %%%%%%%%%%%%%%%%%%%%%%%
    % Configure
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Configure a 2D Gaussian distribution with zero mean and
    % covariance matrix [2.0 0.5
    %                    0.5 1.0]:
    g1 = Gaussian(zeros(2, 1), [2.0 0.5
                                0.5 1.0]);
    
    % If only a vector is passed for the covariance matrix, it is
    % interpreted as the variances of a diagonal covariance matrix:
    g2 = Gaussian(zeros(4, 1), [2 1.5 0.1 3]);
    
    % An already initialized Gaussian can be changed by using its set() method:
    g3 = Gaussian(ones(3, 1), eye(3));
    
    g3.set(0, 5);
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Get Information
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Dimension of the distribution:
    dim = g2.getDim();
    
    % Get mean, covariance matrix and the lower Cholesky decomposition of
    % the distribution's covariance matrix:
    [mean, cov, covSqrt] = g2.getMeanAndCov();
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Draw Random Samples
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Draw eight random samples from g1:
    samples = g1.drawRndSamples(8);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Evaluate Logarithmic Probability Density Function (PDF)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get log PDF values of g1 for the points [2, -1]' and [5 10]':
    logValues = g1.logPdf([ 2  5
                           -1 10]);
    
    % Or plot entire PDF of g1:
    t      = -5:0.1:5;
    [x, y] = meshgrid(t);
    pos    = [x(:)'
              y(:)'];
    
    values = exp(g1.logPdf(pos));
    
    surf(x, y, reshape(values, [length(t) length(t)]), 'EdgeColor', 'None');
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Copy
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Here, g is only a reference to g3 (no data is copied):
    g = g3;
    
    g.set(zeros(2, 1), 3 * eye(2));
    
    % Hence, the above set() call also changes g3:
    [mean, cov] = g3.getMeanAndCov();
    
    % Use the distribution's copy() method for a real copy:
    g = g3.copy();
    
    g.set(1, 3);
    
    % Now, g3 remains unchanged:
    [mean, cov] = g3.getMeanAndCov();
end
