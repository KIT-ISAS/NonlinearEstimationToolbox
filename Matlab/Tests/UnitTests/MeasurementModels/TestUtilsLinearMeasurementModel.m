
classdef TestUtilsLinearMeasurementModel < TestUtilsMeasurementModels
    % Provides test utilities for the LinearMeasurementModel class.
    
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
        function checkUpdate(obj, test, filter, tol)
            obj.hasMeasMatrix = false;
            obj.checkUpdateConfig(false, false, test, filter, tol);
            
            obj.hasMeasMatrix = true;
            obj.checkUpdateConfig(false, false, test, filter, tol);
        end
        
        function checkUpdateStateDecomp(obj, test, filter, tol)
            % State decomposition enabled
            % => no identity measurement matrix possible
            obj.hasMeasMatrix = true;
            obj.checkUpdateConfig(true, false, test, filter, tol);
        end
        
        function checkUpdateMultiMeas(obj, test, filter, tol)
            obj.hasMeasMatrix = false;
            obj.checkUpdateConfig(false, true, test, filter, tol);
            
            obj.hasMeasMatrix = true;
            obj.checkUpdateConfig(false, true, test, filter, tol);
        end
        
        function checkUpdateStateDecompMultiMeas(obj, test, filter, tol)
            % State decomposition enabled
            % => no identity measurement matrix possible
            obj.hasMeasMatrix = true;
            obj.checkUpdateConfig(true, true, test, filter, tol);
        end
        
        
        function checkUpdateKF(obj, test, createFilter)
            configs    = logical(dec2bin(0:15) - '0');
            numConfigs = 16;
            
            for i = 1:numConfigs
                filter = createFilter();
                
                if configs(i, 1)
                    % State decomposition enabled
                    % => no identity measurement matrix possible
                    obj.hasMeasMatrix = true;
                    obj.checkUpdateConfigKF(configs(i, :), test, filter);
                else
                    obj.hasMeasMatrix = false;
                    obj.checkUpdateConfigKF(configs(i, :), test, filter);
                    
                    obj.hasMeasMatrix = true;
                    obj.checkUpdateConfigKF(configs(i, :), test, filter);
                end
            end
        end
    end
    
    methods (Access = 'protected')
        function [initState, measModel, measurements,  ...
                  trueStateMean, trueStateCov, ...
                  trueMeasMean, trueMeasCov, trueCrossCov] = updateConfig(obj, stateDecomp, multiMeas)
            initState = Gaussian(obj.initMean, obj.initCov);
            
            measModel = LinearMeasurementModel();
            
            if obj.hasMeasMatrix
                if stateDecomp
                    measModel.setMeasurementMatrix(obj.measMatrixStateDecomp);
                    mat = [obj.measMatrixStateDecomp zeros(3, 1)];
                else
                    measModel.setMeasurementMatrix(obj.measMatrix);
                    mat = obj.measMatrix;
                end
                
                measModel.setNoise(obj.measNoise3D);
                [noiseMean, noiseCov] = obj.measNoise3D.getMeanAndCov();
                
                if multiMeas
                    trueMeasMean   = mat * obj.initMean + noiseMean;
                    trueMeasMean   = repmat(trueMeasMean, 2, 1);
                    trueMeasCov    = mat * obj.initCov * mat';
                    trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                      trueMeasCov            trueMeasCov + noiseCov];
                    invTrueMeasCov = trueMeasCov \ eye(6);
                    crossCov       = obj.initCov * mat';
                    crossCov       = [crossCov crossCov];
                    
                    measurements = obj.measurements3D;
                else
                    trueMeasMean   = mat * obj.initMean + noiseMean;
                    trueMeasCov    = mat * obj.initCov * mat' + noiseCov;
                    invTrueMeasCov = trueMeasCov \ eye(3);
                    crossCov       = obj.initCov * mat';
                    
                    measurements = obj.measurement3D;
                end
                
                if stateDecomp
                    % If state decomposition is enabled, the true
                    % cross-covariance matirx is
                    trueCrossCov = crossCov(1, :);
                else
                    trueCrossCov = crossCov;
                end
            else
                mat = eye(2);
                
                measModel.setNoise(obj.measNoise2D);
                [noiseMean, noiseCov] = obj.measNoise2D.getMeanAndCov();
                
                if multiMeas
                    trueMeasMean   = mat * obj.initMean + noiseMean;
                    trueMeasMean   = repmat(trueMeasMean, 2, 1);
                    trueMeasCov    = mat * obj.initCov * mat';
                    trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                      trueMeasCov            trueMeasCov + noiseCov];
                    invTrueMeasCov = trueMeasCov \ eye(4);
                    crossCov       = obj.initCov * mat';
                    crossCov       = [crossCov crossCov];
                    
                    measurements = obj.measurements2D;
                else
                    trueMeasMean   = mat * obj.initMean + noiseMean;
                    trueMeasCov    = mat * obj.initCov * mat' + noiseCov;
                    invTrueMeasCov = trueMeasCov \ eye(2);
                    crossCov       = obj.initCov * mat';
                    
                    measurements = obj.measurement2D;
                end
                
                trueCrossCov = crossCov;
            end
            
            K = crossCov * invTrueMeasCov;
            
            trueStateMean = obj.initMean + K * (measurements(:) - trueMeasMean);
            trueStateCov  = obj.initCov  - K * crossCov';
        end
    end
    
    properties (Access = 'private')
        hasMeasMatrix;
    end
    
    properties (Constant, Access = 'private')
        initMean = [0.3 -pi]';
        initCov  = [0.5 0.1
                    0.1 3.0];
        measMatrix = [3    -4
                      pi/4  0
                      0.5   2];
        measMatrixStateDecomp = [ 3
                                 -0.5
                                  3  ];
        measNoise2D = Gaussian([2 -1]', [ 2.0 -0.5
                                         -0.5  1.3]);
        measNoise3D = Gaussian([2 -1 3]', [ 2.0 -0.5 0.2
                                           -0.5  1.3 0.0
                                            0.2  0.0 3.0]);
        measurement2D  = [ 3 -4]';
        measurement3D  = [15 -0.9 -3]';
        measurements2D = [ 3  3.3
                          -4 -3.9];                            
        measurements3D = [ 15  15.2
                          -0.9 -0.8 
                          -3   -3.3];
    end
end
