
classdef GaussianSamplingRnd < GaussianSampling
    % Implements Gaussian random sampling.
    %
    % GaussianSamplingRnd Methods:
    %   GaussianSamplingRnd - Class constructor.
    %   copy                - Copy a GaussianSampling instance.
    %   getStdNormalSamples - Get a set of samples approximating a standard normal distribution.
    %   getSamples          - Get a set of samples approximating a Gaussian distribution.
    %   setNumSamples       - Set the number of samples used to approximate a Gaussian.
    %   getNumSamples       - Get the current number of samples used to approximate a Gaussian.
    
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
        function obj = GaussianSamplingRnd()
            % Class constructor.
            %
            % Returns:
            %   << obj (GaussianSamplingRnd)
            %      A new GaussianSamplingRnd instance.
            
            obj.setNumSamples(100);
        end
        
        function setNumSamples(obj, numSamples)
            % Set the number of samples used to approximate a Gaussian.
            %
            % Parameters:
            %   << numSamples (Positive scalar)
            %      The new number of samples.
            
            if ~Checks.isPosScalar(numSamples)
                error('GaussianSamplesRnd:InvalidNumberOfSamples', ...
                      'numSamples must be a positive scalar.');
            end
            
            obj.numSamples = ceil(numSamples);
        end
        
        function numSamples = getNumSamples(obj)
            % Get the current number of samples used to approximate a Gaussian.
            % 
            % Returns:
            %   >> numSamples (Positive scalar)
            %      The current number of samples.
            
            numSamples = obj.numSamples;
        end        
        
        function [samples, weights, numSamples] = getStdNormalSamples(obj, dimension)
            if ~Checks.isPosScalar(dimension)
                error('GaussianSamplingRnd:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            numSamples = obj.numSamples;
            
            samples = randn(dimension, obj.numSamples);
            
            weights = repmat(1 / obj.numSamples, 1, obj.numSamples);
        end
    end
    
    properties (Access = 'private')
        % The number of samples used to approximate a Gaussian.
        numSamples;
    end
end
