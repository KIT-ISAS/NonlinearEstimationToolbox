
function DiracMixtureExample()
    %%%%%%%%%%%%%%%%%%%%%%%
    % Configure
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Configure a Dirac mixture with 4 components/samples:
    % 1) sample: [2.0, 0.5]', weight: 0.4
    % 2) sample: [-1,  3]',   weight: 0.2
    % 3) sample: [-3, -1]',   weight: 0.2
    % 4) sample: [0, 4]',     weight: 0.2
    samples = [2.0 -1 -3 0
               0.5  3 -1 4];
    weights = [0.4 0.2 0.2 0.2];
    
    dm1 = DiracMixture(samples, weights);
    
    % The component weights do not have to be normalized in advance. That
    % is, the DiracMixture class normalizes the given weights anyway.
    % Hence, this results in the same, valid, Dirac mixture:
    weights = [2 1 1 1];
    
    dm2 = DiracMixture(samples, weights);
    
    % If no weights are provided, the components/samples are assumed to be equally weighted:
    dm3 = DiracMixture(samples);
    
    % An already initialized DiracMixture can be changed by using its set() method:
    dm3.set([-1 5 3 4 2]);
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Get Information
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Dimension of the distribution:
    dim = dm2.getDim();
    
    % Get mean, covariance matrix and the lower Cholesky decomposition of
    % the distribution's covariance matrix:
    [mean, cov, covSqrt] = dm2.getMeanAndCov();
    
    % Get the number of Dirac mixture components/samples:
    numComponents = dm2.getNumComponents();
    
    % Get the Dirac mixture components (note the now normalized weights):
    [samples, weights] = dm2.getComponents();
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Draw Random Samples
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Draw eight random samples from dm1:
    samples = dm1.drawRndSamples(8);
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Copy
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Here, dm is only a reference to dm3 (no data is copied):
    dm = dm3;
    
    dm.set([eye(3) -eye(3)]);
    
    % Hence, the above set() call also changes dm3:
    [mean, cov] = dm3.getMeanAndCov();
    
    % Use the distribution's copy() method for a real copy:
    dm = dm3.copy();
    
    dm.set([1 2 3 4]);
    
    % Now, dm3 remains unchanged:
    [mean, cov] = dm3.getMeanAndCov();
end
