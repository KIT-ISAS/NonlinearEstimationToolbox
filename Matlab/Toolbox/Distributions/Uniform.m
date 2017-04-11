
classdef Uniform < Distribution
    % This class represents a multivariate uniform distribution.
    %
    % Uniform Methods:
    %   Uniform        - Class constructor.
    %   getDim         - Get the dimension of the distribution.
    %   getMeanAndCov  - Get mean and covariance of the distribution.
    %   drawRndSamples - Draw random samples from the distribution.
    %   logPdf         - Evaluate the logarithmic probability density function (pdf) of the distribution.
    %   getInterval    - Get the support of the uniform distribution.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015  Jannik Steinbring <jannik.steinbring@kit.edu>
    %
    %                        Institute for Anthropomatics and Robotics
    %                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
    %                        Karlsruhe Institute of Technology (KIT), Germany
    %
    %                        http://isas.uka.de
    %
    %    This program is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU General Public License as published by
    %    the Free Software Foundation, either version 3 of the License, or
    %    (at your option) any later version.
    %
    %    This program is distributed in the hope that it will be useful,
    %    but WITHOUT ANY WARRANTY; without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    %
    %    You should have received a copy of the GNU General Public License
    %    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    methods
        function obj = Uniform(a, b)
            % Class constructor
            %
            % Parameters
            %   >> a (Vector)
            %      Lower bounds of the multivariate uniform distribution.
            %      Default: 0.
            %
            %   >> b (Vector)
            %      Upper bounds of the multivariate uniform distribution.
            %      Must have the same length as a.
            %      Default: 1.
            
            if nargin == 2
                obj.set(a, b);
            else
                obj.set(0, 1);
            end
            
            obj.mean        = [];
            obj.covariance  = [];
            obj.covSqrt     = [];
        end
        
        function dim = getDim(obj)
            dim = obj.dim;
        end
        
        function [mean, covariance, covSqrt] = getMeanAndCov(obj)
            if isempty(obj.mean)
                obj.mean       = 0.5 * (obj.a + obj.b);
                obj.covariance = diag((obj.b - obj.a).^2 / 12);
            end
            
            mean       = obj.mean;
            covariance = obj.covariance;
            
            if nargout >= 3
                if isempty(obj.covSqrt)
                    obj.covSqrt = sqrt(obj.covariance);
                end
                
                covSqrt = obj.covSqrt;
            end
        end
        
        function rndSamples = drawRndSamples(obj, numSamples)
            if ~Checks.isPosScalar(numSamples)
                error('Uniform:InvalidNumberOfSamples', ...
                      'numSamples must be positive scalar.');
            end
            
            rndSamples = bsxfun(@times, obj.lengths, rand(obj.dim, numSamples));
            rndSamples = bsxfun(@plus, obj.a, rndSamples);
        end
        
        function logValues = logPdf(obj, values)
            obj.checkValues(values);
            
            numValues = size(values, 2);
            
            idxValid                   = zeros(2 * obj.dim, numValues);
            idxValid(1:obj.dim, :)     = bsxfun(@ge, values, obj.a);
            idxValid(obj.dim+1:end, :) = bsxfun(@le, values, obj.b);
            
            idx = all(idxValid, 1);
            
            logValues      = -inf(1, numValues);
            logValues(idx) = obj.validLogValue;
        end
        
        function [a, b] = getInterval(obj)
            % Get the support of the uniform distribution.
            %
            % Returns:
            %   << a (Vector)
            %      Lower bounds of the multivariate uniform distribution.
            %
            %   << b (Vector)
            %      Upper bounds of the multivariate uniform distribution.
            
            a = obj.a;
            b = obj.b;
        end
    end
    
    methods (Access = 'private')
        function set(obj, a, b)
            if ~Checks.isVec(a)
                error('Uniform:InvalidMinimum', ...
                      'a must be vector.');
            end
            
            a = a(:);
            d = size(a, 1);
            
            if ~Checks.isVec(b, d)
                error('Uniform:InvalidMaximum', ...
                      'b must be vector of length %d.', d);
            end
            
            b = b(:);
            
            if any(a >= b)
                error('Uniform:InvalidMinimum', ...
                      'All entries of a must be smaller than their corresponding entries in b.');
            end
            
            obj.a             = a;
            obj.b             = b;
            obj.dim           = d;
            obj.lengths       = obj.b - obj.a;
            obj.validLogValue = -log(prod(obj.lengths));
        end
    end
    
    properties (Access = 'private')
        a;
        b;
        dim;
        lengths;
        validLogValue;
        
        mean;
        covariance;
        covSqrt;
    end
end
