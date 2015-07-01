
classdef PolarMeasModelA < MeasurementModel
    methods
        function measurements = measurementEquation(obj, stateSamples, noiseSamples)
            x = stateSamples(1, :);
            y = stateSamples(2, :);
            
            measurements = [sqrt(x.^2 + y.^2) + noiseSamples(1, :)
                            atan2(y, x)       + noiseSamples(2, :)];
        end
    end
end
