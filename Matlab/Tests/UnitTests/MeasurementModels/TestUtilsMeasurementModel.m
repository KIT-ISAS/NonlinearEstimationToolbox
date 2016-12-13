
classdef TestUtilsMeasurementModel
    % Provides test utilities for the MeasurementModel class.
    
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
        function [measurements, trueMeasMean, ...
                  trueMeasCov, trueCrossCov] = checkUpdate(test, f, tol)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateConfig(false, false, test, f, tol);
        end
        
        function checkUpdateKF(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdate(test, f, tol);
            TestUtilsMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, ...
                                                           trueCrossCov, test, f, tol, numIter);
        end
        
        function [measurements, trueMeasMean, ...
                  trueMeasCov, trueCrossCov] = checkUpdateStateDecomp(test, f, tol)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateConfig(true, false, test, f, tol);
        end
        
        function checkUpdateKFStateDecomp(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateStateDecomp(test, f, tol);
            TestUtilsMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, ...
                                              trueCrossCov, test, f, tol, numIter);
        end
        
        function [measurements, trueMeasMean, ...
                  trueMeasCov, trueCrossCov] = checkUpdateMultiMeas(test, f, tol)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateConfig(false, true, test, f, tol);
        end
        
        function checkUpdateKFMultiMeas(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateMultiMeas(test, f, tol);
            TestUtilsMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, ...
                                              trueCrossCov, test, f, tol, numIter);
        end
        
        function [measurements, trueMeasMean, ...
                  trueMeasCov, trueCrossCov] = checkUpdateMultiMeasStateDecomp(test, f, tol)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateConfig(true, true, test, f, tol);
        end
        
        function checkUpdateKFMultiMeasStateDecomp(test, f, tol, numIter)
            [measurements, trueMeasMean, ...
             trueMeasCov, trueCrossCov] = TestUtilsMeasurementModel.checkUpdateMultiMeasStateDecomp(test, f, tol);
            TestUtilsMeasurementModel.checkKF(measurements, trueMeasMean, trueMeasCov, ...
                                              trueCrossCov, test, f, tol, numIter);
        end
    end
    
    methods (Static, Access = 'private')
        function [measurements, trueMeasMean, ...
                  trueMeasCov, trueCrossCov] = checkUpdateConfig(stateDecomp, multiMeas, test, f, tol)
            measModel = MeasModel(stateDecomp);
            measModel.setNoise(TestUtilsMeasurementModel.measNoise);
            
            if stateDecomp
                mat = [measModel.measMatrix zeros(3, 1)];
            else
                mat = measModel.measMatrix;
            end
            
            [noiseMean, noiseCov] = TestUtilsMeasurementModel.measNoise.getMeanAndCovariance();
            
            if multiMeas
                measurements = [ 1  1.5
                                -2 -1.85
                                 5 -4   ];
                
                trueMeasMean   = mat * TestUtilsMeasurementModel.initMean + noiseMean;
                trueMeasMean   = repmat(trueMeasMean, 2, 1);
                trueMeasCov    = mat * TestUtilsMeasurementModel.initCov * mat';
                trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                  trueMeasCov            trueMeasCov + noiseCov];
                invTrueMeasCov = trueMeasCov \ eye(6);
                crossCov       = TestUtilsMeasurementModel.initCov * mat';
                crossCov       = [crossCov crossCov];
            else
                measurements = [1 -2 5]';
                
                trueMeasMean   = mat * TestUtilsMeasurementModel.initMean + noiseMean;
                trueMeasCov    = mat * TestUtilsMeasurementModel.initCov * mat' + noiseCov;
                invTrueMeasCov = trueMeasCov \ eye(3);
                crossCov       = TestUtilsAdditiveNoiseMeasurementModel.initCov * mat';
            end
            
            K = crossCov * invTrueMeasCov;
            
            trueMean = TestUtilsMeasurementModel.initMean + K * (measurements(:) - trueMeasMean);
            trueCov  = TestUtilsMeasurementModel.initCov - K * crossCov';
            
            if stateDecomp
                % If state decomposition is enabled, the true
                % cross-covariance matirx is
                trueCrossCov = crossCov(1, :);
            else
                trueCrossCov = crossCov;
            end
            
            f.setState(Gaussian(TestUtilsMeasurementModel.initMean, ...
                                TestUtilsMeasurementModel.initCov));
            
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
        initMean  = [0.3 -pi]';
        initCov   = [0.5 0.1; 0.1 3];
        measNoise = Gaussian([2 -1 0.5]', [ 2   -0.5 0
                                           -0.5  1.3 0.5
                                            0    0.5 sqrt(2)]);
    end
end
