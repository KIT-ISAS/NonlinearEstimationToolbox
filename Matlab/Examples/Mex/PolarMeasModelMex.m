
classdef PolarMeasModelMex < AdditiveNoiseMeasurementModel
    methods
        function obj = PolarMeasModelMex()
            measNoise = Gaussian(zeros(2, 1), [1e-2 1e-4]);
            
            obj.setNoise(measNoise);
        end
        
        function measurements = measurementEquation(obj, stateSamples)
            measurements = polarMeasurementEquation(stateSamples);
        end
    end
end
