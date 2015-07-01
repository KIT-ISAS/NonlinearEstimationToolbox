
classdef AddNoiseMeasModel < AdditiveNoiseMeasurementModel
    methods
        function measurements = measurementEquation(obj, stateSamples)
            measurements = obj.measMatrix * stateSamples;
        end
    end
    
    properties (Constant)
        measMatrix = [3 -4
                      0  2
                      3  0];
    end
end
