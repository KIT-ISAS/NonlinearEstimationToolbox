
classdef GaussianSamplingRUKF < GaussianSampling
    % Implements the RUKF Gaussian sampling technique.
    %
    % GaussianSamplingRUKF Methods:
    %   GaussianSamplingRUKF - Class constructor.
    %   copy                 - Copy a GaussianSampling instance.
    %   getStdNormalSamples  - Get a set of samples approximating a standard normal distribution.
    %   getSamples           - Get a set of samples approximating a Gaussian distribution.
    %   setNumIterations     - Set the number of iterations.
    %   getNumIterations     - Get the current number of iterations.
    
    % Literature:
    %   Jindrich Dunik, Ondrej Straka, and Miroslav Simandl,
    %   The Development of a Randomised Unscented Kalman Filter,
    %   Proceedings of the 18th IFAC World Congress,
    %   Milano, Italy, Aug 2011, pp. 8-13.
    
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
        function obj = GaussianSamplingRUKF()
            % Class constructor.
            %
            % Returns:
            %   << obj (GaussianSamplingRUKF)
            %      A new GaussianSamplingRUKF instance.
            
            % By default, 5 iterations are used.
            obj.numIterations = 5;
        end
        
        function setNumIterations(obj, numIterations)
            % Set the number of iterations.
            %
            % By default, 5 iterations are used.
            % 
            % Parameters:
            %   >> numIterations (Positive scalar)
            %      The new number of iterations.
            
            if ~Checks.isPosScalar(numIterations)
                error('GaussianSamplingRUKF:InvalidNumberOfIterations', ...
                      'numIterations must be a positive scalar.');
            end
            
            obj.numIterations = ceil(numIterations);
        end
        
        function numIterations = getNumIterations(obj)
            % Get the current number of iterations.
            %
            % Returns:
            %   << numIterations (Positive scalar)
            %      The current number of iterations.
            
            numIterations = obj.numIterations;
        end
        
        function [samples, weights, numSamples] = getStdNormalSamples(obj, dimension)
            if ~Checks.isPosScalar(dimension)
                error('GaussianSamplingRUKF:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            numIters   = obj.numIterations;
            numSamples = 2 * dimension * numIters + 1;
            
            % Allocate sample & weight memory
            samples = zeros(dimension, numSamples);
            weights = zeros(1, numSamples);
            
            % Generate standard normal samples & sample weights
            step = 2 * dimension - 1;
            a    = 2;
            
            for i = 1:numIters
                U = Utils.rndOrthogonalMatrix(dimension);
                
                scaling = sqrt(chi2rnd(2 + dimension));
                
                b = a + step;
                
                samples(:, a:b) = scaling * [U -U];
                
                weights(1) = weights(1) + (1 - (dimension / scaling^2));
                
                weights(a:b) = (1 / (2 * scaling^2)) * ones(1, 2 * dimension);
                
                a = b + 1;
            end
            
            % Normalize weights
            weights = weights / numIters;
        end
    end
    
    properties (Access = 'private')
        % Number of iterations.
        numIterations;
    end
end
