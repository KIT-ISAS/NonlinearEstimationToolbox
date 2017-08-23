
classdef TargetSysModel < SystemModel
    methods
        function obj = TargetSysModel()
            % Set size of discrete time step
            obj.deltaT = 0.1;
            
            % Set time-invariant, zero-mean Gaussian system noise.
            % Note that the system noise can changed at any time by calling
            % the setNoise() method.
            sysNoise = Gaussian(zeros(2, 1), [1e-1 1e-3]);
            
            obj.setNoise(sysNoise);
        end
        
        % Implement the abstract systemEquation() method inherited from SystemModel
        function predictedStates = systemEquation(obj, stateSamples, noiseSamples)
            numSamples = size(stateSamples, 2);
            
            predictedStates = nan(5, numSamples);
            
            px       = stateSamples(1, :);
            py       = stateSamples(2, :);
            dir      = stateSamples(3, :);
            speed    = stateSamples(4, :);
            turnRate = stateSamples(5, :);
            
            speedNoise    = noiseSamples(1, :);
            turnRateNoise = noiseSamples(2, :);
            
            predSpeed    = speed + speedNoise;
            predTurnRate = turnRate + turnRateNoise;
            predDir      = dir + obj.deltaT .* predTurnRate;
            
            predictedStates(1, :) = px + cos(predDir) .* obj.deltaT .* predSpeed;
            predictedStates(2, :) = py + sin(predDir) .* obj.deltaT .* predSpeed;
            predictedStates(3, :) = predDir;
            predictedStates(4, :) = predSpeed;
            predictedStates(5, :) = predTurnRate;
        end
        
        % Override the derviate() method inherited from SystemModel in
        % order to implement analytic derivatives
        function [stateJacobian, ...
                  noiseJacobian] = derivative(obj, nominalState, nominalNoise)
            dir      = nominalState(3);
            speed    = nominalState(4);
            turnRate = nominalState(5);
            
            speedNoise    = nominalNoise(1);
            turnRateNoise = nominalNoise(2);
            
            t = obj.deltaT;
            
            predSpeed    = speed + speedNoise;
            predTurnRate = turnRate + turnRateNoise;
            predDir      = dir + t * predTurnRate;
            
            a =  cos(predDir) * t;
            b =  sin(predDir) * t;
            c = -b * predSpeed;
            d =  a * predSpeed;
            e =  c * t;
            f =  d * t;
            
            stateJacobian = [1 0 c a e
                             0 1 d b f
                             0 0 1 0 t
                             0 0 0 1 0
                             0 0 0 0 1];
            
            noiseJacobian = [a e
                             b f
                             0 t
                             1 0
                             0 1];
        end
    end
    
    properties (Access = 'private')
        % Stores the step size
        deltaT;
    end
end
