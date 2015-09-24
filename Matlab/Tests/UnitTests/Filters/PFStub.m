
classdef PFStub < PF
    methods
        function obj = PFStub()
            obj = obj@PF('PFStub');
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
