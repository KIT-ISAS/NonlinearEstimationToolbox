
classdef GaussianMixture < Distribution
    % This class represents a multivariate Gaussian mixture distribution.
    % 
    % GaussianMixture Methods:
    %   GaussianMixture      - Class constructor.
    %   getDimension         - Get the dimension of the distribution.
    %   getMeanAndCovariance - Get mean and covariance of the distribution.
    %   drawRndSamples       - Draw random samples from the distribution.
    %   logPdf               - Evaluate the logarithmic probability density function (pdf) of the distribution.
    %   getNumComponents     - Get the number of Gaussian mixture components.
    %   getComponents        - Get the Gaussian mixture components (means, covariances, and weights).
    
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
        function obj = GaussianMixture(means, covariances, weights)
            % Class constructor.
            %
            % Parameters
            %   >> means (Matrix)
            %      Column-wise arranged means of the Gaussian mixture components.
            %      Default: 0.
            %
            %   >> covariances (3D matrix containing positive definite matrices)
            %      Slice-wise arranged covariance matrices of the Gaussian mixture components.
            %      Default: 1.
            %
            %   >> weights (Row vector)
            %      Row-wise arranged weights of the Gaussian mixture components.
            %      Default: All Gaussian mixture components are equally weighted.
            
            if nargin == 3
                obj.set(means, covariances, weights);
            elseif nargin == 2
                obj.set(means, covariances);
            else
                obj.set(0, 1);
            end
            
            obj.mean        = [];
            obj.covariance  = [];
            obj.covSqrt     = [];
            obj.cumWeights  = [];
            obj.logPdfConst = [];
            obj.invCovSqrts = [];
        end
        
        function dimension = getDimension(obj)
            dimension = obj.dimension;
        end
        
        function [mean, covariance, covSqrt] = getMeanAndCovariance(obj)
            if isempty(obj.mean)
                [meanMeans, covMeans] = Utils.getMeanAndCov(obj.means, obj.weights);
                
                obj.mean = meanMeans;
                
                weightedCovs = bsxfun(@times, obj.covariances, ...
                                      reshape(obj.weights, [1 1 obj.numComponents]));
                
                obj.covariance = covMeans + sum(weightedCovs, 3);
            end
            
            mean       = obj.mean;
            covariance = obj.covariance;
            
            if nargout >= 3
                if isempty(obj.covSqrt)
                    obj.covSqrt = chol(obj.covariance)';
                end
                
                covSqrt = obj.covSqrt;
            end
        end
        
        function rndSamples = drawRndSamples(obj, numSamples)
            if ~Checks.isPosScalar(numSamples)
                error('GaussianMixture:InvalidNumberOfSamples', ...
                      'numSamples must be positive scalar.');
            end
            
            if isempty(obj.cumWeights)
                obj.cumWeights = cumsum(obj.weights);
            end
            
            % Select Gaussian components randomly according to the mixture weights
            u = rand(1, numSamples);
            
            u = sort(u);
            
            numComponentSamples = zeros(1, obj.numComponents);
            
            i = 1;
            
            for j = 1:numSamples
                while u(j) > obj.cumWeights(i)
                    i = i + 1;
                end
                
                numComponentSamples(i) = numComponentSamples(i) + 1;
            end
            
            % Generate random samples
            rndSamples = nan(obj.dimension, numSamples);
            
            a = 1;
            b = 0;
            
            for i = 1:obj.numComponents
                numCompSamples = numComponentSamples(i);
                
                if numCompSamples > 0
                    b = b + numCompSamples;
                    
                    rndSamples(:, a:b) = Utils.drawGaussianRndSamples(obj.means(:, i), ...
                                                                      obj.covSqrts(:, :, i), ...
                                                                      numCompSamples);
                    
                    a = b + 1;
                end
            end
        end
        
        function logValues = logPdf(obj, values)
            obj.checkValues(values);
            
            if isempty(obj.logPdfConst)
                logNormConst = obj.dimension * 0.5 * log(2 * pi);
                
                obj.invCovSqrts = nan(obj.dimension, obj.dimension, obj.numComponents);
                obj.logPdfConst = nan(1, obj.numComponents);
                
                logWeights = log(obj.weights);
                
                for i = 1:obj.numComponents
                    obj.invCovSqrts(:, :, i) = obj.covSqrts(:, :, i) \ eye(obj.dimension);
                    
                    logSqrtDetCov = sum(log(diag(obj.covSqrts(:, :, i))));
                    
                    obj.logPdfConst(i) = logWeights(i) - (logSqrtDetCov + logNormConst);
                end
            end
            
            compValues = nan(obj.numComponents, size(values, 2));
            
            for i = 1:obj.numComponents
                s = bsxfun(@minus, values, obj.means(:, i));
                
                v = obj.invCovSqrts(:, :, i) * s;
                
                compValues(i, :) = obj.logPdfConst(i) - 0.5 * sum(v.^2, 1);
            end
            
            maxLogCompValues = max(compValues);
            
            compValues = bsxfun(@minus, compValues, maxLogCompValues);
            
            compValues = exp(compValues);
            
            logValues = maxLogCompValues + log(sum(compValues));
        end
        
        function numComponents = getNumComponents(obj)
            % Get the number of Gaussian mixture components.
            %
            % Returns:
            %   << numComponents (Scalar )
            %      The number of Gaussian mixture components.
            
            numComponents = obj.numComponents;
        end
        
        function [means, covariances, weights] = getComponents(obj)
            % Get the Gaussian mixture components (means, covariances, and weights).
            %
            % Returns:
            %   << means (Matrix)
            %      Column-wise arranged means of the Gaussian mixture components.
            %
            %   << covariances (3D matrix containing positive definite matrices)
            %      Slice-wise arranged covariance matrices of the Gaussian mixture components.
            %
            %   << weights (Row vector)
            %      Row-wise arranged weights of the Gaussian mixture components.
            
            means       = obj.means;
            covariances = obj.covariances;
            weights     = obj.weights;
        end
    end
    
    methods (Access = 'private')
        function set(obj, means, covariances, weights)
            % Check means
            if ~Checks.isMat(means)
                error('GaussianMixture:InvalidMeans', ...
                      'Means must be a matrix.');
            end
            
            [dim, numComps] = size(means);
            
            obj.dimension     = dim;
            obj.numComponents = numComps;
            obj.means         = means;
            
            % Check covariances
            if numComps == 1
                [isCov, obj.covSqrts] = Checks.isCov(covariances, dim);
                
                if ~isCov
                    error('GaussianMixture:InvalidCovariances', ...
                          'Covariances must be a positive definite matrix of dimension %dx%d.', ...
                          dim, dim);
                end
            else
                [isCov3D, obj.covSqrts] = Checks.isCov3D(covariances, dim, numComps);
                
                if ~isCov3D
                    error('GaussianMixture:InvalidCovariances', ...
                          'Covariances must be a matrix of dimension %dx%dx%d containing positive definite matrices.', ...
                          dim, dim, numComps);
                end
            end
            
            obj.covariances = covariances;
            
            % Check weights
            if nargin == 4
                if ~Checks.isNonNegativeRowVec(weights, numComps)
                    error('GaussianMixture:InvalidWeights', ...
                          'Component weights must be a row vector of dimension %d containing only non-negative values.', numComps);
                end
                
                % Normalize component weights
                sumWeights = sum(weights);
                
                if sumWeights <= 0
                    error('GaussianMixture:InvalidWeights', ...
                          'Sum of component weights is not positive.');
                end
                
                obj.weights = weights / sumWeights;
            else
                % Equally weighted components
                obj.weights = repmat(1 / numComps, 1, numComps);
            end
        end
    end
    
    properties (Access = 'private')
        dimension;
        numComponents;
        means;
        covariances;
        covSqrts;
        weights;
        
        mean;
        covariance;
        covSqrt;
        
        cumWeights;
        
        logPdfConst;
        invCovSqrts;
    end
end
