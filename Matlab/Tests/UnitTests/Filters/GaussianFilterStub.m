
classdef GaussianFilterStub < GaussianFilter
    methods
        function obj = GaussianFilterStub()
            obj = obj@GaussianFilter('GaussianFilterStub');
        end
    end
    
    methods (Access = 'protected')
        function predictedMomentsArbitraryNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function predictedMomentsAdditiveNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function performUpdateObservable(obj, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
    end
end
