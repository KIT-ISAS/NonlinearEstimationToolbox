
classdef GaussianSamplingLCD < GaussianSampling
    % Implements the LCD based Gaussian sampling technique.
    %
    % GaussianSamplingLCD Methods:
    %   GaussianSamplingLCD   - Class constructor.
    %   getStdNormalSamples   - Get a set of samples approximating a standard normal distribution.
    %   getSamples            - Get a set of samples approximating a Gaussian distribution.
    %   setNumSamples         - Set an absolute number of samples.
    %   setNumSamplesByFactor - Set a linear factor to determine the number of samples.
    %   setOnlineMode         - Select between online and offline sampling.
    %   setSymmetricMode      - Select between symmetric and asymmetric sampling.
    
    % Literature:
    %   Jannik Steinbring, Martin Pander, Uwe D. Hanebeck,
    %   The Smart Sampling Kalman Filter with Symmetric Samples
    %   arXiv preprint: Systems and Control (cs.SY), June 2015.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   LRKF Revisited: The Smart Sampling Kalman Filter (S²KF),
    %   Journal of Advances in Information Fusion, Vol. 9, No. 2, Dec 2014, pp. 106-123.
    %
    %   Uwe D. Hanebeck, Marco F. Huber, and Vesa Klumpp,
    %   Dirac Mixture Approximation of Multivariate Gaussian Densities,
    %   Proceedings of the 2009 IEEE Conference on Decision and Control (CDC 2009),
    %   Shanghai, China, December 2009.
    
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
            obj.setSymmetricMode(true);
            
            % By default, the offline sample computation is used.
            obj.setOnlineMode(false);
            
            % By default, use a factor of 10 to determine the number of samples.
            obj.setNumSamplesByFactor(10);
        end
        
        function setNumSamples(obj, numSamples)
            % Set an absolute number of samples.
            %
            % This overwrites a possible previous setting, where the number of samples
            % are determined by a linear factor (see setNumSamplesByFactor()).
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
            %    Number of samples = factor * dimension
            %
            % This overwrites a possible previous setting, where the number of samples
            % are determined in an absolute way (see setNumSamples()).
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
        
        function setOnlineMode(obj, onlineMode)
            % Select between online and offline sampling.
            %
            % By default, the offline sample computation, i.e., the sample
            % cache, is used.
            %
            % Parameters:
            %   >> onlineMode (Logical scalar)
            %      If true, the samples will be computed online.
            %      Otherwise, samples from the sample cache will be used.
            
            if ~Checks.isFlag(onlineMode)
            	error('GaussianSamplingLCD:InvalidInput', ...
                      'onlineMode must be a logical scalar.');
            end
            
            if onlineMode
                obj.computingMethod = @obj.computeOnline;
            else
                obj.computingMethod = @obj.computeOffline;
            end
        end
        
        function setSymmetricMode(obj, useSymmetric)
            % Select between symmetric and asymmetric sampling.
            %
            % By default, the symmetric sampling mode is used.
            %
            % Parameters:
            %   >> useSymmetric (Logcial scalar)
     	    %      If true, the symmetric Gaussian LCD sampling scheme is used.
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
        
        function [samples, weights, numSamples] = getStdNormalSamples(obj, dimension)
            if ~Checks.isPosScalar(dimension)
                error('GaussianSamplingLCD:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            numSamples = obj.computeNumSamples(dimension);
            
         	[samples, weights] = obj.computingMethod(dimension, numSamples);
        end
    end
    
    methods (Access = 'private')
        function numSamples = computeNumSamples(obj, dim)
            if isempty(obj.numSamplesAbsolute)
                numSamples = obj.numSamplesFactor * dim;
                
                if mod(numSamples, 2) == 0
                    numSamples = numSamples + 1;
                end
            else
                numSamples = obj.numSamplesAbsolute;
            end
        end
        
        function [samples, weights] = computeOffline(obj, dim, numSamples)
            [samples, weights] = obj.sampleCache.getSamples(dim, numSamples);
        end
        
        function [samples, weights] = computeOnline(obj, dim, numSamples)
            if obj.useSymmetric
                if numSamples < 2 * dim
                    error('GaussianSamplingLCD:InvalidNumberOfSamples', ...
                          'The number of samples must be at least twice the dimension.');
                end
            else
                if numSamples <= dim
                    error('GaussianSamplingLCD:InvalidNumberOfSamples', ...
                          'The number of samples must be greater than the dimension.');
                end
            end
            
            compute = true;
            
            while compute
                try
                    [samples, weight] = GLCD(dim, numSamples, obj.useSymmetric);
                    weights = repmat(weight, 1, numSamples);
                    
                    compute = false;
                catch
                    warning('GaussianSamplingLCD:ComputingSamplesFailed', ...
                            'Computing Gaussian LCD samples failed. Trying again.');
                end
            end
        end
    end
    
    properties (Access = 'private')
        % Symmetric / Asymmetric GLCD sample cache.
        sampleCache;
        
        % Absolute number of samples.
        numSamplesAbsolute; 
        
        % Linear factor to determine the number of samples.
        numSamplesFactor;
        
     	% If true, the symmetric Gaussian LCD sampling scheme is used.
        % Otherwise, the asymmetric one is used.
        useSymmetric;
        
        % Function handle to select between online computation and sample
        % lookup using the sample cache (offline computation).
        computingMethod;    
    end
end
