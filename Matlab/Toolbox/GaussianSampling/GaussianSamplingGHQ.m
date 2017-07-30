
classdef GaussianSamplingGHQ < GaussianSampling
    % Implements the Gauss-Hermite quadrature technique.
    %
    % GaussianSamplingGHQ Methods:
    %   GaussianSamplingGHQ    - Class constructor.
    %   copy                   - Copy a GaussianSampling instance.
    %   getStdNormalSamples    - Get a set of samples approximating a standard normal distribution.
    %   getSamples             - Get a set of samples approximating a Gaussian distribution.
    %   setNumQuadraturePoints - Set the number of quadrature points.
    %   getNumQuadraturePoints - Get the number of quadrature points.
    
    % Literature:
    %   Kazufumi Ito and Kaiqi Xiong,
    %   Gaussian Filters for Nonlinear Filtering Problems,
    %   IEEE Transactions on Automatic Control, Vol. 45, No. 5, May 2000, pp. 910-927.
    
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
        function obj = GaussianSamplingGHQ()
            % Class constructor.
            %
            % Returns:
            %   << obj (GaussianSamplingGHQ)
            %      A new GaussianSamplingGHQ instance.
            
            % Call superclass constructor
            obj = obj@GaussianSampling();
            
            obj.sampleCache = SampleCacheGHQ();
        end
        
        function setNumQuadraturePoints(obj, numPoints)
            % Set the number of quadrature points.
            %
            % By default, 2 quadrature points are used.
            %
            % Parameters:
            %   >> numPoints (Scalar in { 2, 3, 4 })
            %      The new number of quadrature points.
            
            obj.sampleCache.setNumQuadraturePoints(numPoints);
        end
        
        function numPoints = getNumQuadraturePoints(obj)
            % Get the number of quadrature points.
            %
            % Returns:
            %   << numPoints (Scalar in { 2, 3, 4 })
            %      The number of quadrature points.
            
            numPoints = obj.sampleCache.getNumQuadraturePoints();
        end
        
        function [samples, weights, numSamples] = getStdNormalSamples(obj, dimension)
            if ~Checks.isPosScalar(dimension)
                error('GaussianSamplingGHQ:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            numPoints  = obj.sampleCache.getNumQuadraturePoints();
            numSamples = numPoints^dimension;
            
            [samples, weights] = obj.sampleCache.getSamples(dimension, numSamples);
        end
    end
    
    methods (Access = 'protected')
        function cpObj = copyElement(obj)
            cpObj = obj.copyElement@GaussianSampling();
            
            cpObj.sampleCache = obj.sampleCache.copy();
        end
    end
    
    properties (Access = 'private')
        % Gaussian Hermite quadrature sample cache.
        sampleCache;
    end
end
