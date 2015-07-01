
classdef AddNoiseSysModel < AdditiveNoiseSystemModel
    methods
        function predictedStates = systemEquation(obj, stateSamples)
            predictedStates = obj.sysMatrix * stateSamples;
        end
    end
    
    properties (Constant)
        sysMatrix = [3 -4; 0 2];
    end
end
