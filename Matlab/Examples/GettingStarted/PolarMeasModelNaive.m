
classdef PolarMeasModelNaive < MeasurementModel
    methods
        function obj = PolarMeasModelNaive()
            % Set time-invariant, zero-mean Gaussian measurement noise.
            % Note that the measurement noise can changed at any time by
            % calling the setNoise() method.
            measNoise = Gaussian(zeros(2, 1), [1e-2 1e-4]);
            
            obj.setNoise(measNoise);
        end
        
        % Implement the abstract measurementEquation() method
        % inherited from MeasurementModel
        function measurements = measurementEquation(obj, stateSamples, noiseSamples)
            px = stateSamples(1, :);
            py = stateSamples(2, :);
            
            measurements = [sqrt(px.^2 + py.^2)
                            atan2(py, px)      ] + noiseSamples;
        end
    end
end
