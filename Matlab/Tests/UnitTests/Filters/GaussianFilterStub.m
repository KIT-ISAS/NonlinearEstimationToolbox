
classdef GaussianFilterStub < GaussianFilter
    methods
        function obj = GaussianFilterStub()
            obj = obj@GaussianFilter('GaussianFilterStub');
        end
    end
    
    methods (Access = 'protected')
        function performPrediction(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function performUpdate(obj, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
    end
end
