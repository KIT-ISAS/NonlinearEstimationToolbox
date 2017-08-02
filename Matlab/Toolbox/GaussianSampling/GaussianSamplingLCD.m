
classdef GaussianSamplingLCD < GaussianSampling
    % Implements the LCD-based Gaussian sampling technique.
    %
    % GaussianSamplingLCD Methods:
    %   GaussianSamplingLCD   - Class constructor.
    %   copy                  - Copy a GaussianSampling instance.
    %   getStdNormalSamples   - Get a set of samples approximating a standard normal distribution.
    %   getSamples            - Get a set of samples approximating a Gaussian distribution.
    %   setNumSamples         - Set an absolute number of samples.
    %   setNumSamplesByFactor - Set a linear factor to determine the number of samples.
    %   getNumSamplesConfig   - Get the number of samples configuration.
    %   setSymmetricMode      - Select between point-symmetric and asymmetric sampling.
    %   getSymmetricMode      - Get the selected sampling mode.
    
    % Literature:
    %   Jannik Steinbring, Martin Pander, and Uwe D. Hanebeck,
    %   The Smart Sampling Kalman Filter with Symmetric Samples
    %   Journal of Advances in Information Fusion, Vol. 11, No. 1, Jun 2016, pp. 71-90.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   LRKF Revisited: The Smart Sampling Kalman Filter (S²KF),
    %   Journal of Advances in Information Fusion, Vol. 9, No. 2, Dec 2014, pp. 106-123.
    %
    %   Uwe D. Hanebeck, Marco F. Huber, and Vesa Klumpp,
    %   Dirac Mixture Approximation of Multivariate Gaussian Densities,
    %   Proceedings of the 2009 IEEE Conference on Decision and Control (CDC 2009),
    %   Shanghai, China, Dec 2009.
    
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
        function obj = GaussianSamplingLCD()
            % Class constructor.
            %
            % Returns:
            %   << obj (GaussianSamplingLCD)
            %      A new GaussianSamplingLCD instance.
            
            % By default, the symmetric sampling mode is used.
            obj.useSymmetric = true;
            obj.sampleCache  = SampleCacheGLCDSym();
            
            % By default, use a factor of 10 to determine the number of samples.
            obj.numSamplesAbsolute = [];
            obj.numSamplesFactor   = 10;
        end
        
        function setNumSamples(obj, numSamples)
            % Set an absolute number of samples.
            %
            % This also overwrites a possible previous setting, where the number of
            % samples are determined by a linear factor (see setNumSamplesByFactor()).
            %
            % By default, a linear factor 10 is used.
            %
            % Parameters:
            %    >> numSamples (Positive scalar)
            %       The new absolute number of samples.
            
            if ~Checks.isPosScalar(numSamples)
                error('GaussianSamplingLCD:InvalidNumberOfSamples', ...
                      'numSamples must be a positive scalar.');
            end
            
            obj.numSamplesAbsolute = ceil(numSamples);
            obj.numSamplesFactor   = [];
        end
        
        function setNumSamplesByFactor(obj, factor)
            % Set a linear factor to determine the number of samples.
            %
            % The actual number of samples will be computed according to
            %
            %    Number of samples = factor * dimension + 1 - mod(factor * dimension, 2)
            %
            % i.e., always an odd number of samples is used.
            %
            % This also overwrites a possible previous setting, where the number of
            % samples are determined in an absolute way (see setNumSamples()).
            %
            % By default, a linear factor 10 is used.
            %
            % Parameters:
            %    >> factor (Positive scalar)
            %       The new linear factor to determine the number of samples.
            
            if ~Checks.isPosScalar(factor)
                error('GaussianSamplingLCD:InvalidSampleFactor', ...
                      'factor must be a positive scalar.');
            end
            
            obj.numSamplesAbsolute = [];
            obj.numSamplesFactor   = ceil(factor);
        end
        
        function [numSamplesAbs, ...
                  numSamplesFactor] = getNumSamplesConfig(obj)
            % Get the number of samples configuration.
            %
            % Returns:
            %   << numSamplesAbs (Positive scalar or empty matrix)
            %      Equals the absolute number of samples if set.
            %      Otherwise, an empty matrix.
            %
            %   << numSamplesFactor (Positive scalar or empty matrix)
            %      Equals the sample factor if set.
            %      Otherwise, an empty matrix.
            
            numSamplesAbs    = obj.numSamplesAbsolute;
            numSamplesFactor = obj.numSamplesFactor;
        end
        
        function setSymmetricMode(obj, useSymmetric)
            % Select between point-symmetric and asymmetric sampling.
            %
            % By default, the recommended point-symmetric sampling scheme is used.
            %
            % Parameters:
            %   >> useSymmetric (Logcial scalar)
            %      If true, the point-symmetric sampling scheme is used.
            %      Otherwise, the asymmetric one is used.
            
            if ~Checks.isFlag(useSymmetric)
                error('GaussianSamplingLCD:InvalidInput', ...
                      'useSymmetric must be a logical scalar.');
            end
            
            obj.useSymmetric = useSymmetric;
            
            if obj.useSymmetric
                obj.sampleCache = SampleCacheGLCDSym();
            else
                obj.sampleCache = SampleCacheGLCDAsym();
            end
        end
        
        function useSymmetric = getSymmetricMode(obj)
            % Get the selected sampling mode.
            %
            % Returns:
            %   << useSymmetric (Logical scalar)
            %      If true, the point-symmetric sampling scheme is used.
            %      Otherwise, the asymmetric one is used.
            
            useSymmetric = obj.useSymmetric;
        end
        
        function [samples, weights, numSamples] = getStdNormalSamples(obj, dimension)
            if ~Checks.isPosScalar(dimension)
                error('GaussianSamplingLCD:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            numSamples = obj.computeNumSamples(dimension);
            
            [samples, weights] = obj.sampleCache.getSamples(dimension, numSamples);
        end
    end
    
    methods (Access = 'private')
        function numSamples = computeNumSamples(obj, dim)
            if isempty(obj.numSamplesAbsolute)
                numSamples = obj.numSamplesFactor * dim;
                
                numSamples = numSamples + 1 - mod(numSamples, 2);
            else
                numSamples = obj.numSamplesAbsolute;
            end
        end
    end
    
    methods (Access = 'protected')
        function cpObj = copyElement(obj)
            cpObj = obj.copyElement@GaussianSampling();
            
            cpObj.sampleCache        = obj.sampleCache.copy();
            cpObj.numSamplesAbsolute = obj.numSamplesAbsolute;
            cpObj.numSamplesFactor   = obj.numSamplesFactor;
            cpObj.useSymmetric       = obj.useSymmetric;
        end
    end
    
    properties (Access = 'private')
        % Symmetric/Asymmetric Gaussian LCD sample cache.
        sampleCache;
        
        % Absolute number of samples.
        numSamplesAbsolute;
        
        % Linear factor to determine the number of samples.
        numSamplesFactor;
        
        % If true, the point-symmetric LCD-based Gaussian sampling scheme is used.
        % Otherwise, the asymmetric one is used.
        useSymmetric;
    end
end
