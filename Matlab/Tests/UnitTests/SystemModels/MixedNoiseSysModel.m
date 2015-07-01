
classdef MixedNoiseSysModel < MixedNoiseSystemModel
    methods
        function predictedStates = systemEquation(obj, stateSamples, noiseSamples)
            predictedStates = obj.sysMatrix * stateSamples + noiseSamples;
        end
    end
    
    properties (Constant)
        sysMatrix = [3 -4; 0 2];
    end
end
