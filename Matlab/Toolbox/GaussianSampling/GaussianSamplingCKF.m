
classdef GaussianSamplingCKF < GaussianSampling
    % Implements the fifth-degree cubature Kalman filter sampling technique.
    %
    % GaussianSamplingCKF Methods:
    %   GaussianSamplingCKF - Class constructor.
    %   copy                - Copy a GaussianSampling instance.
    %   getStdNormalSamples - Get a set of samples approximating a standard normal distribution.
    %   getSamples          - Get a set of samples approximating a Gaussian distribution.
    
    % Literature:
    %   Bin Jia, Ming Xin, and Yang Cheng,
    %   High-Degree Cubature Kalman Filter,
    %   Automatica, Vol. 49, No. 2, Feb 2013, pp. 510-518.
    
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
        function obj = GaussianSamplingCKF()
            % Class constructor.
            %
            % Returns:
            %   << obj (GaussianSamplingCKF)
            %      A new GaussianSamplingCKF instance.
            
            % Call superclass constructor
            obj = obj@GaussianSampling();
            
            obj.sampleCache = SampleCacheCKF();
        end
        
        function [samples, weights, numSamples] = getStdNormalSamples(obj, dimension)
            if ~Checks.isPosScalar(dimension)
                error('GaussianSamplingCKF:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            numSamples = 2 * dimension^2 + 1;
            
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
        % Five degree CKF sample cache.
        sampleCache;
    end
end
