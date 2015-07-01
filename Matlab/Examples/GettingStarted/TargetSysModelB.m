
classdef TargetSysModelB < AdditiveNoiseSystemModel 
    methods
        function predictedStates = systemEquation(obj, stateSamples)
            [dimState, numSamples] = size(stateSamples);
            
            x      = stateSamples(1, :);
            y      = stateSamples(2, :);
            phi    = stateSamples(3, :);
            s      = stateSamples(4, :);
            dotphi = stateSamples(5, :);
            
            predictedStates = nan(dimState, numSamples);
            
            % Predict x-position
            predictedStates(1, :) = x + obj.deltaT * s .* cos(phi);
            
            % Predict y-position
            predictedStates(2, :) = y + obj.deltaT * s .* sin(phi);
            
            % Predict orientation
            predictedStates(3, :) = phi + obj.deltaT * dotphi;
            
            % Predict speed
            predictedStates(4, :) = s;
            
            % Predict angular velocity
            predictedStates(5, :) = dotphi;
        end
        
        function stateJacobian = derivative(obj, nominalState)
            phi = nominalState(3);
            s   = nominalState(4);
            t   = obj.deltaT;
            
            stateJacobian = [1 0 -t * s .* sin(phi) t * cos(phi) 0
                             0 1  t * s .* cos(phi) t * sin(phi) 0
                             0 0  1                 0            t
                             0 0  0                 1            0
                             0 0  0                 0            1];
        end
    end
    
    properties
        % Stores the step size
        deltaT;
    end
end
