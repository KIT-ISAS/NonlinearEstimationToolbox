
classdef LikelihoodExample < Likelihood
    methods
        function obj = LikelihoodExample()
            obj.noise = Gaussian(zeros(2, 1), [1e-2, 1e-4]);
        end
        
        function logValues = logLikelihood(obj, stateSamples, measurement)
            x = stateSamples(1, :);
            y = stateSamples(2, :);
            
            % Evaluate deterministic part of the measurement model
            h = [sqrt(x.^2 + y.^2)
                 atan2(y, x)      ];
            
            % Compute differences y - h(x)
            diffs = bsxfun(@minus, measurement, h);
            
            % Evaluate the measurement noise logarithmic pdf
            logValues = obj.noise.logPdf(diffs);
        end
    end
    
    properties (Access = 'private')
        noise;
    end
end
