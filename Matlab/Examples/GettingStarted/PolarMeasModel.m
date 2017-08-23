
classdef PolarMeasModel < AdditiveNoiseMeasurementModel
    methods
        function obj = PolarMeasModel()
            % Set time-invariant, zero-mean Gaussian measurement noise.
            % Note that the measurement noise can changed at any time by
            % calling the setNoise() method.
            measNoise = Gaussian(zeros(2, 1), [1e-2 1e-4]);
            
            obj.setNoise(measNoise);
        end
        
        % Implement the abstract measurementEquation() method
        % inherited from AdditiveNoiseMeasurementModel
        function measurements = measurementEquation(obj, stateSamples)
            px = stateSamples(1, :);
            py = stateSamples(2, :);
            
            measurements = [sqrt(px.^2 + py.^2)
                            atan2(py, px)      ];
        end
        
        % Override the derviate() method inherited from
        % AdditiveNoiseMeasurementModel in order to
        % implement the analytic derivative
        function stateJacobian = derivative(obj, nominalState)
            px = nominalState(1);
            py = nominalState(2);
            
            a = px^2 + py^2;
            b = sqrt(a);
            
            stateJacobian = [ px / b, py / b, 0, 0, 0
                             -py / a, px / a, 0, 0, 0];
        end
    end
end
