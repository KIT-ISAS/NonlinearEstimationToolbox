
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
        
        function predictedMomentsMixedNoise(obj, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
        
        function performUpdate(obj, ~, ~)
            obj.error('NotImplemented', 'Not implemented');
        end
    end
end
