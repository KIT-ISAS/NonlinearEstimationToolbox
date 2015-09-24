
classdef SampleBasedGaussianFilterStub < SampleBasedGaussianFilter
    methods
        function obj = SampleBasedGaussianFilterStub()
            obj = obj@SampleBasedGaussianFilter('SampleBasedGaussianFilterStub');
        end
    end
    
    methods (Access = 'protected')
        function predictArbitraryNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function predictAdditiveNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function predictMixedNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function performUpdate(obj, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
    end
end
