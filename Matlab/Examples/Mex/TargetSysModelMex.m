
classdef TargetSysModelMex < SystemModel
    methods
        function obj = TargetSysModelMex()
            obj.deltaT = 0.1;
            
            sysNoise = Gaussian(zeros(2, 1), [1e-1 1e-3]);
            
            obj.setNoise(sysNoise);
        end
        
        function predictedStates = systemEquation(obj, stateSamples, noiseSamples)
            predictedStates = targetSystemEquation(stateSamples, noiseSamples, obj.deltaT);
        end
    end
    
    properties (Access = 'private')
        deltaT;
    end
end
