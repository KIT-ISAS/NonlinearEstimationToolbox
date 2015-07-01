
classdef TestMixedNoiseMeasurementModel < matlab.unittest.TestCase
    % Provides unit tests for the MixedNoiseMeasurementModel class.
    
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
    
    methods (Test)
        function testSimulateDefaultNumMeasurements(obj)
        	measModel = MixedNoiseMeasModel();
            measModel.setAdditiveNoise(Uniform([0 0 0], [1 1 1]));
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            detMeas = measModel.measMatrix * obj.initMean;
            
            measurements = measModel.simulate(obj.initMean);
            
            obj.verifyEqual(size(measurements), [3 1]);
            obj.verifyGreaterThanOrEqual(measurements, detMeas);
            obj.verifyLessThanOrEqual(measurements, detMeas + 2);
        end
        
        function testSimulate(obj)
        	measModel = MixedNoiseMeasModel();
            measModel.setAdditiveNoise(Uniform([0 0 0], [1 1 1]));
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            detMeas = measModel.measMatrix * obj.initMean;
            
            n = 3;
            
            measurements = measModel.simulate(obj.initMean, n);
            
            obj.verifyEqual(size(measurements), [3 n]);
            obj.verifyGreaterThanOrEqual(measurements, repmat(detMeas, 1, n));
            obj.verifyLessThanOrEqual(measurements, repmat(detMeas, 1, n) + 2);
        end
        
        function testEKF(obj)
            f = EKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testEKFMultiIter(obj)
            f = EKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testEKFMultiMeas(obj)
            f = EKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testEKFMultiMeasMultiIter(obj)
            f = EKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testCKF(obj)
            f = CKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1); 
        end
        
        function testCKFMultiIter(obj)
            f = CKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3); 
        end
        
        function testCKFMultiMeas(obj)
            f = CKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testCKFMultiMeasMultiIter(obj)
            f = CKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testGHKF(obj)
            f = GHKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1); 
        end
        
        function testGHKFMultiIter(obj)
            f = GHKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3); 
        end
        
        function testGHKFMultiMeas(obj)
            f = GHKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testGHKFMultiMeasMultiIter(obj)
            f = GHKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testRUKF(obj)
            f = RUKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1); 
        end
        
        function testRUKFMultiIter(obj)
            f = RUKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3); 
        end
        
        function testRUKFMultiMeas(obj)
            f = RUKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testRUKFMultiMeasMultiIter(obj)
            f = RUKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testS2KF(obj)
            f = S2KF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1); 
        end
        
        function testS2KFMultiIter(obj)
            f = S2KF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3); 
        end
        
        function testS2KFMultiMeas(obj)
            f = S2KF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testS2KFMultiMeasMultiIter(obj)
            f = S2KF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testUKF(obj)
            f = UKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1); 
        end
        
        function testUKFMultiIter(obj)
            f = UKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdate(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3); 
        end
        
        function testUKFMultiMeas(obj)
            f = UKF();
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 1);
        end
        
        function testUKFMultiMeasMultiIter(obj)
            f = UKF();
            f.setMaxNumIterations(3);
            
            [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = obj.checkUpdateMultiMeas(f);
            
            obj.checkKFUpdate(measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, 3);
        end
        
        function testEnKF(obj)
            f = EnKF();
            f.setEnsembleSize(5000000);
            
            obj.checkUpdate(f, 5 * 1e-2);
        end
        
        function testEnKFMultiMeas(obj)
            f = EnKF();
            f.setEnsembleSize(5000000);
            
            obj.checkUpdateMultiMeas(f, 5 * 1e-2);
        end
    end
    
    methods (Access = 'private')
        function [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = checkUpdate(obj, f, tol)
            if nargin < 3
                tol = sqrt(eps);
            end
            
        	measModel = MixedNoiseMeasModel();
            measModel.setAdditiveNoise(obj.addMeasNoise);
            measModel.setNoise(obj.measNoise);
            
            mat                         = measModel.measMatrix;
            [addNoiseMean, addNoiseCov] = obj.addMeasNoise.getMeanAndCovariance();
            [noiseMean, noiseCov]       = obj.measNoise.getMeanAndCovariance();
            
            measurements = [1 -2 5]';
            
            trueMeasMean = mat * obj.initMean + addNoiseMean + noiseMean;
            trueMeasCov  = mat * obj.initCov * mat' + addNoiseCov + noiseCov;
            trueCrossCov = obj.initCov * mat';
            
            invMeasCov = trueMeasCov \ eye(3);
            
            K = trueCrossCov * invMeasCov;
            
            trueMean = obj.initMean + K * (measurements - trueMeasMean);
            trueCov  = obj.initCov - K * trueCrossCov';
            
            f.setState(Gaussian(obj.initMean, obj.initCov));
            
            f.update(measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            obj.verifyEqual(mean, trueMean, 'RelTol', tol);
            obj.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        function [measurements, trueMeasMean, trueMeasCov, trueCrossCov] = checkUpdateMultiMeas(obj, f, tol)
            if nargin < 3
                tol = sqrt(eps);
            end
            
        	measModel = MixedNoiseMeasModel();
            measModel.setAdditiveNoise(obj.addMeasNoise);
            measModel.setNoise(obj.measNoise);
            
            mat                         = measModel.measMatrix;
            [addNoiseMean, addNoiseCov] = obj.addMeasNoise.getMeanAndCovariance();
            [noiseMean, noiseCov]       = obj.measNoise.getMeanAndCovariance();
            
            measurements = [ 1  1.5
                            -2 -1.85
                             5 -4   ];
            
            trueMeasMean = mat * obj.initMean + addNoiseMean + noiseMean;
            trueMeasMean = repmat(trueMeasMean, 2, 1);
            trueMeasCov  = mat * obj.initCov * mat';
            trueMeasCov  = [trueMeasCov + addNoiseCov + noiseCov trueMeasCov
                            trueMeasCov                          trueMeasCov + addNoiseCov + noiseCov];
            trueCrossCov = obj.initCov * mat';
            trueCrossCov = [trueCrossCov trueCrossCov];
            
            invMeasCov = trueMeasCov \ eye(6);
            
            K = trueCrossCov * invMeasCov;
            
            trueMean = obj.initMean + K * (measurements(:) - trueMeasMean);
            trueCov  = obj.initCov - K * trueCrossCov';
            
            f.setState(Gaussian(obj.initMean, obj.initCov));
            
            f.update(measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            obj.verifyEqual(mean, trueMean, 'RelTol', tol);
            obj.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        function checkKFUpdate(obj, measurements, trueMeasMean, trueMeasCov, trueCrossCov, f, numIter)
         	tol = sqrt(eps);
            
            [meas, ...
             measMean, ...
             measCov, ...
             stateMeasCrossCov, ...
             numIterations] = f.getLastUpdateData();
            
            obj.verifyEqual(meas, measurements(:), 'RelTol', tol);
            obj.verifyEqual(measMean, trueMeasMean, 'RelTol', tol);
            obj.verifyEqual(measCov, trueMeasCov, 'RelTol', tol);
            obj.verifyEqual(stateMeasCrossCov, trueCrossCov, 'RelTol', tol);
            
            obj.verifyEqual(numIterations, numIter);
        end
    end
    
    properties (Constant)
        initMean     = [0.3 -pi]';
        initCov      = [0.5 0.1; 0.1 3];
        addMeasNoise = Gaussian([2 -1 0.5]', [ 2   -0.5 0
                                              -0.5  1.3 0.5
                                               0    0.5 sqrt(2)]);
        measNoise    = Gaussian([1 2.5 0]', diag([1 2 3]));
    end
end
