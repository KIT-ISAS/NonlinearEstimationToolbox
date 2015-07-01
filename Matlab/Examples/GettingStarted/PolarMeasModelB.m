
classdef PolarMeasModelB < AdditiveNoiseMeasurementModel
    methods
        function measurements = measurementEquation(obj, stateSamples)
            x = stateSamples(1, :);
            y = stateSamples(2, :);
            
            measurements = [sqrt(x.^2 + y.^2)
                            atan2(y, x)      ];
        end
        
        function stateJacobian = derivative(obj, nominalState)
            x = nominalState(1);
            y = nominalState(2);
            
            a = x^2 + y^2;
            b = sqrt(a);
            
            stateJacobian = [ x / b, y / b, 0, 0, 0
                             -y / a, x / a, 0, 0, 0];
        end
    end
end
