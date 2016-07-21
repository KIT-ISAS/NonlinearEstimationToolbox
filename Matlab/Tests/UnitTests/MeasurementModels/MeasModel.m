
classdef MeasModel < MeasurementModel
    methods
        function obj = MeasModel(stateDecomp)
            if nargin < 1
                stateDecomp = false;
            end
            
            if stateDecomp
                obj.measMatrix = [ 3
                                  -0.5
                                   3  ];
            else
                obj.measMatrix = [3 -4
                                  0  2
                                  3  0];
            end
        end
        
        function measurements = measurementEquation(obj, stateSamples, noiseSamples)
            measurements = obj.measMatrix * stateSamples + noiseSamples;
        end
    end
    
    properties (SetAccess = 'private', GetAccess = 'public')
        measMatrix;
    end
end
