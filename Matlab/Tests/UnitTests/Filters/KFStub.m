
classdef KFStub < KF
    methods
        function obj = KFStub()
            obj = obj@KF('KFStub');
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
