
classdef PolarMeasLikelihood < Likelihood
    methods
        function obj = PolarMeasLikelihood()
            % Time-invariant, zero-mean Gaussian measurement noise
            obj.measNoise = Gaussian(zeros(2, 1), [1e-2 1e-4]);
        end
        
        % Implement the abstract logLikelihood() method inherited from Likelihood
        function logValues = logLikelihood(obj, stateSamples, measurement)
            px = stateSamples(1, :);
            py = stateSamples(2, :);
            
            % Evaluate deterministic part of the measurement model
            h = [sqrt(px.^2 + py.^2)
                 atan2(py, px)      ];
            
            % Compute differences y - h(x)
            diffs = bsxfun(@minus, measurement, h);
            
            % Evaluate the measurement noise logarithmic Gaussian PDF
            logValues = obj.measNoise.logPdf(diffs);
        end
    end
    
    properties (Access = 'private')
        measNoise;
    end
end
