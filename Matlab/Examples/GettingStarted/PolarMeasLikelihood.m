
classdef PolarMeasLikelihood < Likelihood
    methods
        function setNoise(obj, noise)
            obj.measNoise = noise;
        end
        
        function logValues = logLikelihood(obj, stateSamples, measurements)
            x = stateSamples(1, :);
            y = stateSamples(2, :);
            
            % Evaluate deterministic part of the measurement model
            h = [sqrt(x.^2 + y.^2)
                 atan2(y, x)      ];
             
           	% Compute differences y - h(x)
            diffs = bsxfun(@minus, measurements, h);
            
            % Evaluate the measurement noise logarithmic pdf
            logValues = obj.measNoise.logPdf(diffs);
        end
    end
    
    properties (Access = 'private')
        measNoise;
    end
end