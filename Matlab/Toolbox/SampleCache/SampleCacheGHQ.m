
classdef SampleCacheGHQ < SampleCache
    % A sample cache for the Gauss-Hermite quadrature.
    %
    % SampleCacheGHQ Methods:
    %   SampleCacheGHQ         - Class constructor.
    %   copy                   - Copy a SampleCache instance.
    %   getSamples             - Retrieve samples from the sample cache.
    %   generateSamples        - Generate and store samples in the sample cache.
    %   setNumQuadraturePoints - Set the number of quadrature points.
    %   getNumQuadraturePoints - Get the current number of quadrature points.
    
    % Literature:
    %   Kazufumi Ito, Kaiqi Xiong,
    %   Gaussian Filters for Nonlinear Filtering Problems,
    %   IEEE Transactions on Automatic Control, Vol. 45, Issue 5, Pages 910 - 927, May, 2000.
    
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
        function obj = SampleCacheGHQ()
            % Class constructor.
            %
            % Returns:
            %   << obj (SampleCacheGHQ)
            %      A new SampleCacheGHQ instance.
            
            % Construct path
            file = mfilename('fullpath');
            path = fileparts(file); 
            
            sampleCachePath = [path '/' SampleCacheGHQ.sampleCacheDir];
            
            % Call superclass constructor
            obj = obj@SampleCache(sampleCachePath);
            
            % By default, 2 quadrature points are used.
            obj.setNumQuadraturePoints(2);
        end
        
        function setNumQuadraturePoints(obj, numPoints)
            % Set the number of quadrature points.
            %
            % By default, 2 quadrature points are used.
            %
            % Parameters:
            %   >> numPoints (Scalar in { 2, 3, 4 })
            %      The new number of quadrature points.
            
            if ~Checks.isScalar(numPoints) || ...
               ~(numPoints == 2 || numPoints == 3 || numPoints == 4)
                error('SampleCacheGHQ:InvalidNumberOfQuadraturePoints', ...
                      'numPoints must be in { 2, 3, 4 }.');
            end
            
            if numPoints ~= obj.getNumQuadraturePoints()
                switch (numPoints)   
                    case 2
                        a = 1;
                        obj.oneDimSamples = [a -a];
                        
                        a = 0.5;
                        obj.oneDimWeights = [a a];
                    case 3
                        a = 0;
                        b = sqrt(3);
                        obj.oneDimSamples = [a b -b];
                        
                        a = 2/3;
                        b = 1/6;
                        obj.oneDimWeights = [a b b];
                    case 4
                        a = sqrt(3 - sqrt(6));
                        b = sqrt(3 + sqrt(6));
                        obj.oneDimSamples = [a -a b -b];
                        
                        a = 1 / (4 * (3 - sqrt(6)));
                        b = 1 / (4 * (3 + sqrt(6)));
                        obj.oneDimWeights = [a a b b];
                end
            end
        end
        
        function numPoints = getNumQuadraturePoints(obj)
            % Get the current number of quadrature points.
            %
            % Returns:
            %   << numPoints (Scalar in { 2, 3, 4 })
            %      The current number of quadrature points.
            
            numPoints = size(obj.oneDimSamples, 2);
        end
    end
    
    methods (Access = 'protected')
        function checkDimAndNumSamplesCombination(obj, dimension, numSamples)
            numPoints          = obj.getNumQuadraturePoints();
            expectedNumSamples = numPoints^dimension;
            
            if numSamples ~= expectedNumSamples
                error('SampleCacheGHQ:InvalidNumberOfSamples', ...
                      'Dimension %d requires %d samples.', ...
                      dimension, expectedNumSamples);
            end
        end
        
        function [samples, weights] = computeSamples(obj, dim, ~)
            samples = obj.cartesianProduct(obj.oneDimSamples, dim);
            
            weights = obj.cartesianProduct(obj.oneDimWeights, dim);
            weights = prod(weights, 1);
        end
    end
    
    methods (Static, Access = 'private')
        function cartProd = cartesianProduct(data, dim)
            g = cell(dim, 1);
            
            [g{:}] = ndgrid(data);
            
            cartProd = zeros(dim, length(data)^dim);
            
            for i = 1:dim
               cartProd(i, :) = g{i}(:)'; 
            end
        end
    end
    
    properties (Constant, Access = 'private')
        sampleCacheDir = 'SampleCacheGHQ';
    end
    
    properties (Access = 'private')
        % Samples approximating 1D standard normal distribution.
        oneDimSamples;
        
        % Corresponding sample weights approximating 1D standard normal distribution.
        oneDimWeights;
    end
end