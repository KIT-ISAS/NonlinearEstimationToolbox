
classdef TargetSysModelA < SystemModel
    methods
        function predictedStates = systemEquation(obj, stateSamples, noiseSamples)
            [dimState, numSamples] = size(stateSamples);
            
            x      = stateSamples(1, :);
            y      = stateSamples(2, :);
            phi    = stateSamples(3, :);
            s      = stateSamples(4, :);
            dotphi = stateSamples(5, :);
            
            predictedStates = nan(dimState, numSamples);
            
            % Predict x-position
            predictedStates(1, :) = x + obj.deltaT * s .* cos(phi) + noiseSamples(1, :);
            
            % Predict y-position
            predictedStates(2, :) = y + obj.deltaT * s .* sin(phi) + noiseSamples(2, :);
            
            % Predict orientation
            predictedStates(3, :) = phi + obj.deltaT * dotphi + noiseSamples(3, :);
            
            % Predict speed
            predictedStates(4, :) = s + noiseSamples(4, :);
            
            % Predict angular velocity
            predictedStates(5, :) = dotphi + noiseSamples(5, :);
        end
    end
    
    properties
        % Stores the step size
        deltaT;
    end
end
