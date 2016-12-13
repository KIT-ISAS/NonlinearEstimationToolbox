
classdef TestUtilsLinearMeasurementModel
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
    
    methods (Static)
        function checkUpdate(test, f, tol)
            TestUtilsLinearMeasurementModel.checkUpdateConfig(false, false, false, test, f, tol);
            TestUtilsLinearMeasurementModel.checkUpdateConfig(true, false, false, test, f, tol);
        end
        
        function checkUpdateKF(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsLinearMeasurementModel.checkUpdateConfig(false, false, false, test, f, tol);
            TestUtilsLinearMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter);
            
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsLinearMeasurementModel.checkUpdateConfig(true, false, false, test, f, tol);
            TestUtilsLinearMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter);
        end
        
        function checkUpdateStateDecomp(test, f, tol)
            TestUtilsLinearMeasurementModel.checkUpdateConfig(true, false, true, test, f, tol);
        end
        
        function checkUpdateKFStateDecomp(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsLinearMeasurementModel.checkUpdateConfig(true, false, true, test, f, tol);
            TestUtilsLinearMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter);
        end
        
        function checkUpdateMultiMeas(test, f, tol)
            TestUtilsLinearMeasurementModel.checkUpdateConfig(false, true, false, test, f, tol);
            TestUtilsLinearMeasurementModel.checkUpdateConfig(true, true, false, test, f, tol);
        end
        
        function checkUpdateKFMultiMeas(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsLinearMeasurementModel.checkUpdateConfig(false, true, false, test, f, tol);
            TestUtilsLinearMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter);
            
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsLinearMeasurementModel.checkUpdateConfig(true, true, false, test, f, tol);
            TestUtilsLinearMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter);
        end
        
        function checkUpdateMultiMeasStateDecomp(test, f, tol)
            TestUtilsLinearMeasurementModel.checkUpdateConfig(true, true, true, test, f, tol);
        end
        
        function checkUpdateKFMultiMeasStateDecomp(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsLinearMeasurementModel.checkUpdateConfig(true, true, true, test, f, tol);
            TestUtilsLinearMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter);
        end
    end
    
    methods (Static, Access = 'private')
        function [measurements, trueMeasMean, ...
                  trueMeasCov, trueCrossCov] = checkUpdateConfig(measMatrix, multiMeas, stateDecomp, test, f, tol)
            measModel = LinearMeasurementModel();
            
            if measMatrix
                if stateDecomp
                    measModel.setMeasurementMatrix(TestUtilsLinearMeasurementModel.measMatrixStateDecomp);
                    mat = [TestUtilsLinearMeasurementModel.measMatrixStateDecomp zeros(3, 1)];
                else
                    measModel.setMeasurementMatrix(TestUtilsLinearMeasurementModel.measMatrix);
                    mat = TestUtilsLinearMeasurementModel.measMatrix;
                end
                
                measModel.setNoise(TestUtilsLinearMeasurementModel.measNoise3D);
                [noiseMean, noiseCov] = TestUtilsLinearMeasurementModel.measNoise3D.getMeanAndCovariance();
                
                if multiMeas
                    trueMeasMean   = mat * TestUtilsLinearMeasurementModel.initMean + noiseMean;
                    trueMeasMean   = repmat(trueMeasMean, 2, 1);
                    trueMeasCov    = mat * TestUtilsLinearMeasurementModel.initCov * mat';
                    trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                      trueMeasCov            trueMeasCov + noiseCov];
                    invTrueMeasCov = trueMeasCov \ eye(6);
                    crossCov       = TestUtilsLinearMeasurementModel.initCov * mat';
                    crossCov       = [crossCov crossCov];
                    
                    measurements = TestUtilsLinearMeasurementModel.measurements3D;
                else
                    trueMeasMean   = mat * TestUtilsLinearMeasurementModel.initMean + noiseMean;
                    trueMeasCov    = mat * TestUtilsLinearMeasurementModel.initCov * mat' + noiseCov;
                    invTrueMeasCov = trueMeasCov \ eye(3);
                    crossCov       = TestUtilsLinearMeasurementModel.initCov * mat';
                    
                    measurements = TestUtilsLinearMeasurementModel.measurement3D;
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
                
                measModel.setNoise(TestUtilsLinearMeasurementModel.measNoise2D);
                [noiseMean, noiseCov] = TestUtilsLinearMeasurementModel.measNoise2D.getMeanAndCovariance();
                
                if multiMeas
                    trueMeasMean   = mat * TestUtilsLinearMeasurementModel.initMean + noiseMean;
                    trueMeasMean   = repmat(trueMeasMean, 2, 1);
                    trueMeasCov    = mat * TestUtilsLinearMeasurementModel.initCov * mat';
                    trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                      trueMeasCov            trueMeasCov + noiseCov];
                    invTrueMeasCov = trueMeasCov \ eye(4);
                    crossCov       = TestUtilsLinearMeasurementModel.initCov * mat';
                    crossCov       = [crossCov crossCov];
                    
                    measurements = TestUtilsLinearMeasurementModel.measurements2D;
                else
                    trueMeasMean   = mat * TestUtilsLinearMeasurementModel.initMean + noiseMean;
                    trueMeasCov    = mat * TestUtilsLinearMeasurementModel.initCov * mat' + noiseCov;
                    invTrueMeasCov = trueMeasCov \ eye(2);
                    crossCov       = TestUtilsLinearMeasurementModel.initCov * mat';
                    
                    measurements = TestUtilsLinearMeasurementModel.measurement2D;
                end
                
                trueCrossCov = crossCov;
            end
            
            K = crossCov * invTrueMeasCov;
            
            trueMean = TestUtilsLinearMeasurementModel.initMean + K * (measurements(:) - trueMeasMean);
            trueCov  = TestUtilsLinearMeasurementModel.initCov  - K * crossCov';
            
            f.setState(Gaussian(TestUtilsAdditiveNoiseMeasurementModel.initMean, ...
                                TestUtilsAdditiveNoiseMeasurementModel.initCov));
            
            f.update(measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, cov');
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        function checkKF(measurements, trueMeasMean, trueMeasCov, trueCrossCov, test, f, tol, numIter)
            [meas, ...
             measMean, ...
             measCov, ...
             stateMeasCrossCov, ...
             numIterations] = f.getLastUpdateData();
            
            test.verifyEqual(meas, measurements(:), 'RelTol', tol);
            test.verifyEqual(measMean, trueMeasMean, 'RelTol', tol);
            test.verifyEqual(measCov, measCov');
            test.verifyEqual(measCov, trueMeasCov, 'RelTol', tol);
            test.verifyEqual(stateMeasCrossCov, trueCrossCov, 'RelTol', tol);
            
            test.verifyEqual(numIterations, numIter);
        end
    end
    
    properties (Constant)
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
