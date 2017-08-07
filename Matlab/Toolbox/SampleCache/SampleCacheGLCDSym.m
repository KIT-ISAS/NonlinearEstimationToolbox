
classdef SampleCacheGLCDSym < SampleCache
    % A sample cache for the point-symmetric LCD-based Gaussian sampling.
    %
    % SampleCacheGLCDSym Methods:
    %   SampleCacheGLCDSym - Class constructor.
    %   copy               - Copy a SampleCache instance.
    %   getSamples         - Retrieve samples from the sample cache.
    %   generateSamples    - Generate and store samples in the sample cache.
    
    % Literature:
    %   Jannik Steinbring, Martin Pander, and Uwe D. Hanebeck,
    %   The Smart Sampling Kalman Filter with Symmetric Samples
    %   Journal of Advances in Information Fusion, vol. 11, no. 1, pp. 71-90, Jun. 2016.
    
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
        function obj = SampleCacheGLCDSym()
            % Class constructor.
            %
            % Returns:
            %   << obj (SampleCacheGLCDSym)
            %      A new SampleCacheGLCDSym instance.
            
            % Construct path
            file = mfilename('fullpath');
            path = fileparts(file); 
            
            sampleCachePath = [path '/' SampleCacheGLCDSym.sampleCacheDir];
            
            % Call superclass constructor
            obj = obj@SampleCache(sampleCachePath);
        end
    end
    
    methods (Access = 'protected')
        function checkDimAndNumSamplesCombination(~, dimension, numSamples)
            if numSamples < 2 * dimension
                error('SampleCacheGLCDSym:InvalidNumberOfSamples', ...
                      'The number of samples must be at least twice the dimension.');
            end
        end
        
        function [samples, weights] = computeSamples(~, dimension, numSamples)
            compute = true;
            
            while compute
                try
                    [samples, weight] = GLCD(dimension, numSamples, true);
                    weights = repmat(weight, 1, numSamples);
                    
                    compute = false;
                catch ex
                    if strcmp(ex.identifier, 'GLCD:ComputationFailed')
                        % If computation failed, issue a warning and try again
                        warning('SampleCacheGLCDSym:ComputingGLCDFailed', ...
                                'Computing Gaussian LCD samples failed. Trying again.');
                    else
                        % Forward all other exceptions
                        rethrow(ex);
                    end
                end
            end
        end
    end
    
    properties (Constant, Access = 'private')
        sampleCacheDir = 'SampleCacheGLCDSym';
    end
end
