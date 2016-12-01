
classdef DiracMixture < Distribution
    % This class represents a multivariate Dirac mixture (sample) distribution.
    %
    % DiracMixture Methods:
    %   DiracMixture         - Class constructor.
    %   getDimension         - Get the dimension of the distribution.
    %   getMeanAndCovariance - Get mean and covariance of the distribution.
    %   drawRndSamples       - Draw random samples from the distribution.
    %   logPdf               - Evaluate the logarithmic probability density function (pdf) of the distribution.
    %   getNumComponents     - Get the number of Dirac mixture components (samples).
    %   getComponents        - Get the Dirac mixture components (sample positions and weights).
    
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
        function obj = DiracMixture(samples, weights)
            % Class constructor.
            %
            % Parameters
            %   >> samples (Matrix)
            %      Column-wise arranged sample positions.
            %
            %   >> weights (Row vector)
            %      Row-wise arranged weights of the samples.
            %      If no weights are passed, all samples are assumed to be equally weighted.
            
            if nargin == 2
                obj.set(samples, weights)
            else
                obj.set(samples);
            end
            
            obj.mean       = [];
            obj.covariance = [];
            obj.cumWeights = [];
            obj.covSqrt    = [];
        end
        
        function dimension = getDimension(obj)
            dimension = obj.dimension;
        end
        
        function [mean, covariance, covSqrt] = getMeanAndCovariance(obj)
            if isempty(obj.mean)
                [obj.mean, obj.covariance] = Utils.getMeanAndCov(obj.samples, obj.weights);
            end
            
            mean       = obj.mean;
            covariance = obj.covariance;
            
            if nargout >= 3
                if isempty(obj.covSqrt)
                    obj.covSqrt = chol(obj.covariance, 'Lower');
                end
                
                covSqrt = obj.covSqrt;
            end
        end
        
        function rndSamples = drawRndSamples(obj, numSamples)
            if ~Checks.isPosScalar(numSamples)
                error('DiracMixture:InvalidNumberOfSamples', ...
                      'numSamples must be a positive scalar.');
            end
            
            if isempty(obj.cumWeights)
                obj.cumWeights = cumsum(obj.weights);
            end
            
            rndSamples = Utils.resampling(obj.samples, obj.cumWeights, numSamples);
        end
        
        function logPdf(~, ~)
            error('DiracMixture:LogPdfNotSupported', ...
                  'A Dirac mixture has no proper pdf.');
        end
        
        function numComponents = getNumComponents(obj)
            % Get the number of Dirac mixture components (samples).
            %
            % Returns:
            %   << numComponents (Scalar )
            %      The number of Dirac mixture components.
            
            numComponents = obj.numComponents;
        end
        
        function [samples, weights] = getComponents(obj)
            % Get the Dirac mixture components (sample positions and weights).
            %
            % Returns:
            %   << samples (Matrix)
            %      Column-wise arranged sample positions.
            %
            %   << weights (Row vector)
            %      Row-wise arranged weights of the samples.
            
            samples = obj.samples;
            weights = obj.weights;
        end
    end
    
    methods (Access = 'private')
        function set(obj, samples, weights)
            if ~Checks.isMat(samples)
                error('DiracMixture:InvalidSamples', ...
                      'Samples must be a matrix.');
            end
            
            [dim, L] = size(samples);
            
            if nargin == 3
                if ~Checks.isNonNegativeRowVec(weights, L)
                    error('DiracMixture:InvalidWeights', ...
                          'Sample weights must be a row vector of dimension %d containing only non-negative values.', L);
                end
                
                % Normalize sample weights
                sumWeights = sum(weights);
                
                if sumWeights <= 0
                    error('DiracMixture:InvalidWeights', ...
                          'Sum of sample weights is not positive.');
                end
                
                weights = weights / sumWeights;
            else
                % Equally weighted samples
                weights = repmat(1 / L, 1, L);
            end
            
            obj.dimension     = dim;
            obj.numComponents = L;
            obj.samples       = samples;
            obj.weights       = weights;
        end
    end
    
    properties (Access = 'private')
        dimension;
        numComponents;
        samples;
        weights;
        
        mean;
        covariance;
        covSqrt;
        
        cumWeights;
    end
end
