
function GaussianMixtureExample()
    %%%%%%%%%%%%%%%%%%%%%%%
    % Configure
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Configure a Gaussian mixture with 3 components:
    % 1) mean: [2.0, 0.5]', covariance: eye(2),     weight: 0.5
    % 2) mean: [-1,  3]',   covariance: [2.0 0.5
    %                                    0.5 1.0],  weight: 0.25
    % 3) mean: [-3, -1]',   covariance: 3 * eye(2), weight: 0.25
    means   = [2.0 -1 -3
               0.5  3 -1];
    covs    = cat(3, eye(2), [2.0 0.5; 0.5 1.0], 3 * eye(2));
    weights = [0.5 0.25 0.25];
    
    gm1 = GaussianMixture(means, covs, weights);
    
    % The component weights do not have to be normalized in advance. That
    % is, the GaussianMixture class normalizes the given weights anyway.
    % Hence, this results in the same, valid, Gaussian mixture:
    weights = [2 1 1];
    
    gm2 = GaussianMixture(means, covs, weights);
    
    % If no weights are provided, the components are assumed to be equally weighted:
    gm3 = GaussianMixture(means, covs);
    
    % A Gaussian mixture can of course only have a single component
    gm4 = GaussianMixture(ones(3, 1), eye(3));
    
    % An already initialized GaussianMixture can be changed by using its set() method:
    gm4.set([ones(3, 1), -5 * ones(3, 1)], cat(3, eye(3), 5 * eye(3)), [0.8 0.2]);
    
    gm4.set([ones(3, 1), -5 * ones(3, 1)], cat(3, eye(3), 5 * eye(3)));
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Get Information
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Dimension of the distribution:
    dim = gm2.getDim();
    
    % Get mean, covariance matrix and the lower Cholesky decomposition of
    % the distribution's covariance matrix:
    [mean, cov, covSqrt] = gm2.getMeanAndCov();
    
    % Get the number of Gaussian mixture components:
    numComponents = gm2.getNumComponents();
    
    % Get the Gaussian mixture components (note the now normalized weights):
    [means, covs, weights] = gm2.getComponents();
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Draw Random Samples
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Draw eight random samples from gm1:
    samples = gm1.drawRndSamples(8);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Evaluate Logarithmic Probability Density Function (PDF)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get log PDF values of gm1 for the points [2, -1]' and [5 10]':
    logValues = gm1.logPdf([ 2  5
                            -1 10]);
    
    % Or plot entire PDF of gm1:
    t      = -5:0.1:5;
    [x, y] = meshgrid(t);
    pos    = [x(:)'
              y(:)'];
    
    values = exp(gm1.logPdf(pos));
    
    surf(x, y, reshape(values, [length(t) length(t)]), 'EdgeColor', 'None');
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Copy
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Here, gm is only a reference to gm3 (no data is copied):
    gm = gm3;
    
    gm.set(zeros(2, 1), 3 * eye(2));
    
    % Hence, the above set() call also changes gm3:
    [mean, cov] = gm3.getMeanAndCov();
    
    % Use the distribution's copy() method for a real copy:
    gm = gm3.copy();
    
    gm.set([1 2 3], cat(3, 2, 3, 5));
    
    % Now, gm3 remains unchanged:
    [mean, cov] = gm3.getMeanAndCov();
end
