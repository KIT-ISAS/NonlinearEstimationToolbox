
classdef TestUtilsLinearMeasurementModel
    % Provides test utilities for the LinearMeasurementModel class.
    
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
    
    methods (Static)
        function [initState, measModel, ...
                  measurement, stateDecompDim, ...
                  trueStateMean, trueStateCov, ...
                  trueMeasMean, trueMeasCov] = getMeasModelData(hasMeasMatrix, stateDecomp)
            if nargin < 2
                stateDecomp = false;
            end
            
            initState = Gaussian(TestUtilsLinearMeasurementModel.initMean, ...
                                 TestUtilsLinearMeasurementModel.initCov);
            
            measModel = LinearMeasurementModel();
            
            if hasMeasMatrix
                if stateDecomp
                    stateDecompDim = 1;
                    
                    measModelMat = TestUtilsLinearMeasurementModel.measMatrixStateDecomp;
                    measModel.setMeasurementMatrix(measModelMat);
                    mat = [measModelMat zeros(3, 1)];
                else
                    stateDecompDim = 0;
                    
                    measModelMat = TestUtilsLinearMeasurementModel.measMatrix;
                    measModel.setMeasurementMatrix(measModelMat);
                    mat = measModelMat;
                end
                
                measModel.setNoise(TestUtilsLinearMeasurementModel.measNoise3D);
                [noiseMean, noiseCov] = TestUtilsLinearMeasurementModel.measNoise3D.getMeanAndCov();
                
                trueMeasMean   = mat * TestUtilsLinearMeasurementModel.initMean + noiseMean;
                trueMeasCov    = mat * TestUtilsLinearMeasurementModel.initCov * mat' + noiseCov;
                invTrueMeasCov = trueMeasCov \ eye(3);
                crossCov       = TestUtilsLinearMeasurementModel.initCov * mat';
                
                measurement = TestUtilsLinearMeasurementModel.meas3D;
            else
                % Identity measurement matrix => no state decomposition is possible
                stateDecompDim = 0;
                
                mat = eye(2);
                
                measModel.setNoise(TestUtilsLinearMeasurementModel.measNoise2D);
                [noiseMean, noiseCov] = TestUtilsLinearMeasurementModel.measNoise2D.getMeanAndCov();
                
                trueMeasMean   = mat * TestUtilsLinearMeasurementModel.initMean + noiseMean;
                trueMeasCov    = mat * TestUtilsLinearMeasurementModel.initCov * mat' + noiseCov;
                invTrueMeasCov = trueMeasCov \ eye(2);
                crossCov       = TestUtilsLinearMeasurementModel.initCov * mat';
                
                measurement = TestUtilsLinearMeasurementModel.meas2D;
            end
            
            K = crossCov * invTrueMeasCov;
            
            trueStateMean = TestUtilsLinearMeasurementModel.initMean + K * (measurement - trueMeasMean);
            trueStateCov  = TestUtilsLinearMeasurementModel.initCov  - K * crossCov';
        end
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
        meas2D = [ 3 -4]';
        meas3D = [15 -0.9 -3]';
    end
end
