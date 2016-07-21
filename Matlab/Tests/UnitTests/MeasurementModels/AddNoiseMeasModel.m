
classdef AddNoiseMeasModel < AdditiveNoiseMeasurementModel
    methods
        function obj = AddNoiseMeasModel(stateDecomp)
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
        
        function measurements = measurementEquation(obj, stateSamples)
            measurements = obj.measMatrix * stateSamples;
        end
    end
    
    properties (SetAccess = 'private', GetAccess = 'public')
        measMatrix;
    end
end
