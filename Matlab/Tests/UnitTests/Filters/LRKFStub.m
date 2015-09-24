
classdef LRKFStub < LRKF
    methods
        function obj = LRKFStub()
            obj = obj@LRKF('LRKFStub', GaussianSamplingStub(), GaussianSamplingStub());
        end
    end
end
