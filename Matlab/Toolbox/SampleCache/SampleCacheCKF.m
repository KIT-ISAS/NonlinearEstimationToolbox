
classdef SampleCacheCKF < SampleCache
    % A sample cache for the fifth-degree cubature Kalman filter sampling.
    %
    % SampleCacheCKF Methods:
    %   SampleCacheCKF  - Class constructor.
    %   copy            - Copy a SampleCache instance.
    %   getSamples      - Retrieve samples from the sample cache.
    %   generateSamples - Generate and store samples in the sample cache.
    
    % Literature:
    %   Bin Jia, Ming Xin, and Yang Cheng,
    %   High-Degree Cubature Kalman Filter,
    %   Automatica, vol. 49, no. 2, pp. 510-518, Feb. 2013.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
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
        function obj = SampleCacheCKF()
            % Class constructor.
            %
            % Returns:
            %   << obj (SampleCacheCKF)
            %      A new SampleCacheCKF instance.
            
            % Construct path
            file = mfilename('fullpath');
            path = fileparts(file);
            
            sampleCachePath = [path '/' SampleCacheCKF.sampleCacheDir];
            
            % Call superclass constructor
            obj = obj@SampleCache(sampleCachePath);
        end
    end
    
    methods (Access = 'protected')
        function checkDimAndNumSamplesCombination(~, dimension, numSamples)
            expectedNumSamples = 2 * dimension^2 + 1;
            
            if numSamples ~= expectedNumSamples
                error('SampleCacheCKF:InvalidNumberOfSamples', ...
                      'Dimension %d requires %d samples.', ...
                      dimension, expectedNumSamples);
            end
        end
        
        function [samples, weights] = computeSamples(obj, dim, ~)
            [axesSamples, axesWeights]       = obj.computeAxesSamples(dim);
            [offAxesSamples, offAxesWeights] = obj.computeOffAxesSamples(dim);
            
            samples = [axesSamples offAxesSamples];
            weights = [axesWeights offAxesWeights];
        end
    end
    
    methods (Static, Access = 'private')
        function [samples, weights] = computeAxesSamples(dim)
            samples = sqrt(dim + 2) * [zeros(dim, 1) -eye(dim) eye(dim)];
            weights = [2 / (dim + 2) repmat((4 - dim) / (2 * (dim + 2)^2), 1, 2 * dim)];
        end
        
        function [samples, weights] = computeOffAxesSamples(dim)
            n = dim * (dim - 1) * 0.5;
            s1 = zeros(dim, n);
            s2 = zeros(dim, n);
            
            e = eye(dim);
            
            i = 1;
            
            for l = 1:dim
                for k = 1:(l-1)
                    s1(:, i) = e(:, k) + e(:, l);
                    s2(:, i) = e(:, k) - e(:, l);
                    i = i + 1;
                end
            end
            
            samples = sqrt((dim + 2) / 2) * [s1 s2 -s1 -s2];
            weights = repmat(1 / (dim + 2)^2, 1, 4 * n);
        end
    end
    
    properties (Constant, Access = 'private')
        sampleCacheDir = 'SampleCacheCKF';
    end
end