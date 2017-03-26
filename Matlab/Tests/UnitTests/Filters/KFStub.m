
classdef KFStub < KF
    methods
        function obj = KFStub()
            obj = obj@KF('KFStub');
        end
    end
    
    methods (Access = 'protected')
        function predictedMomentsArbitraryNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function predictedMomentsAdditiveNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function getMomentFuncArbitraryNoise(~, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function getMomentFuncAdditiveNoise(~, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function getMomentFuncMixedNoise(~, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
    end
end
