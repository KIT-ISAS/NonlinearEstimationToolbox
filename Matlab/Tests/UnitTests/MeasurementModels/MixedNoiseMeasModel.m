
classdef MixedNoiseMeasModel < MixedNoiseMeasurementModel
    methods
        function measurements = measurementEquation(obj, stateSamples, noiseSamples)
            measurements = obj.measMatrix * stateSamples + noiseSamples;
        end
    end
    
    properties (Constant)
        measMatrix = [3 -4
                      0  2
                      3  0];
    end
end
