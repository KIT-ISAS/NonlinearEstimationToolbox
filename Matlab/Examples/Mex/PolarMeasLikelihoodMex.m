
classdef PolarMeasLikelihoodMex < Likelihood
    methods
        function obj = PolarMeasLikelihoodMex()
            % Inverse covariance matrix of time-invariant,
            % zero-mean Gaussian measurement noise
            obj.invNoiseCov = diag(1 ./ [1e-2 1e-4]);
        end
        
        function logValues = logLikelihood(obj, stateSamples, measurement)
            logValues = polarMeasLogLikelihood(stateSamples, obj.invNoiseCov, measurement);
        end
    end
    
    properties (Access = 'private')
        invNoiseCov;
    end
end
