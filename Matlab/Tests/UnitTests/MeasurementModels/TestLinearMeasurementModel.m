
classdef TestLinearMeasurementModel < matlab.unittest.TestCase
    % Provides unit tests for the LinearMeasurementModel class.
    
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
        function testDefaultConstructor(obj)
            measModel = LinearMeasurementModel();
            
            obj.verifyEqual(measModel.measMatrix, []);
            obj.verifyEqual(measModel.noise, []);
        end
        
        function testConstructorMeasMatrix(obj)
            measModel = LinearMeasurementModel(ones(2, 3));
            
            obj.verifyEqual(measModel.measMatrix, ones(2, 3));
            obj.verifyEqual(measModel.noise, []);
        end
        
        
        function testSetMeasurementMatrix(obj)
            measModel = LinearMeasurementModel();
            
            measModel.setMeasurementMatrix(eye(3));
            
            obj.verifyEqual(measModel.measMatrix, eye(3));
            obj.verifyEqual(measModel.noise, []);
        end
        
        function testSetSetMeasurementMatrixEmpty(obj)
            measModel = LinearMeasurementModel();
            
            measModel.setMeasurementMatrix([]);
            
            obj.verifyEqual(measModel.measMatrix, []);
            obj.verifyEqual(measModel.noise, []);
        end

        
        function testSimulateDefaultNumMeasurements(obj)
            measModel = LinearMeasurementModel(obj.measMatrix);
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            trueState = [sqrt(2) exp(1)]';
            
            detMeas = obj.measMatrix * trueState;
            
            measurements = measModel.simulate(trueState);
            
            obj.verifyEqual(size(measurements), [3 1]);
            obj.verifyGreaterThanOrEqual(measurements, detMeas);
            obj.verifyLessThanOrEqual(measurements, detMeas + 1);
        end
        
        function testSimulate(obj)
            measModel = LinearMeasurementModel(obj.measMatrix);
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            trueState = [sqrt(2) exp(1)]';
            
            detMeas = obj.measMatrix * trueState;
            
            n = 3;
            
            measurements = measModel.simulate(trueState, n);
            
            obj.verifyEqual(size(measurements), [3 n]);
            obj.verifyGreaterThanOrEqual(measurements, repmat(detMeas, 1, n));
            obj.verifyLessThanOrEqual(measurements, repmat(detMeas, 1, n) + 1);
        end
        
        
        function testNonMeasMatrixOneMeasNonAnalytic(obj)
            obj.checkUpdate(false, false, false);
        end
        
        function testMeasMatrixOneMeasNonAnalytic(obj)
            obj.checkUpdate(true, false, false);
        end
        
        function testNoMeasMatrixMultiMeasNonAnalytic(obj)
            obj.checkUpdate(false, true, false);
        end
        
        function testMeasMatrixMultiMeasNonAnalytic(obj)
            obj.checkUpdate(true, true, false);
        end
        

        function testNoMeasMatrixOneMeasAnalytic(obj)
            obj.checkUpdate(false, false, true);
        end
        
        function testMeasMatrixOneMeasAnalytic(obj)
            obj.checkUpdate(true, false, true);
        end
        
        function testNoMeasMatrixMultiMeasAnalytic(obj)
            obj.checkUpdate(false, true, true);
        end
        
        function testMeasMatrixMultiMeasAnalytic(obj)
            obj.checkUpdate(true, true, true);
        end
    end
    
    methods (Access = 'private')
        function checkUpdate(obj, measMatrix, multiMeas, analytic)
        	measModel = LinearMeasurementModel();
            
            if measMatrix
                measModel.setMeasurementMatrix(obj.measMatrix);
                measModel.setNoise(obj.measNoise3D);
                
                [noiseMean, noiseCov] = obj.measNoise3D.getMeanAndCovariance();
                
                if multiMeas
                    trueMeasMean   = obj.measMatrix * obj.initMean + noiseMean;
                    trueMeasMean   = repmat(trueMeasMean, 2, 1);
                    trueMeasCov    = obj.measMatrix * obj.initCov * obj.measMatrix';
                    trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                      trueMeasCov            trueMeasCov + noiseCov];
                    trueCrossCov   = obj.initCov * obj.measMatrix';
                    trueCrossCov   = [trueCrossCov trueCrossCov];
                    invTrueMeasCov = trueMeasCov \ eye(6);
                    
                    meas = obj.measurements3D;
                else
                    trueMeasMean   = obj.measMatrix * obj.initMean + noiseMean;
                    trueMeasCov    = obj.measMatrix * obj.initCov * obj.measMatrix' + noiseCov;
                    trueCrossCov   = obj.initCov * obj.measMatrix';
                    invTrueMeasCov = trueMeasCov \ eye(3);
                    
                    meas = obj.measurement3D;
                end
            else
             	measModel.setNoise(obj.measNoise2D);
                
                [noiseMean, noiseCov] = obj.measNoise2D.getMeanAndCovariance();
                
                if multiMeas
                    trueMeasMean   = obj.initMean + noiseMean;
                    trueMeasMean   = repmat(trueMeasMean, 2, 1);
                    trueMeasCov    = [obj.initCov + noiseCov obj.initCov
                                      obj.initCov            obj.initCov + noiseCov];
                    trueCrossCov   = [obj.initCov obj.initCov];
                    invTrueMeasCov = trueMeasCov \ eye(4);
                    
                    meas = obj.measurements2D;
                else
                    trueMeasMean   = obj.initMean + noiseMean;
                    trueMeasCov    = obj.initCov + noiseCov;
                    trueCrossCov   = obj.initCov;
                    invTrueMeasCov = trueMeasCov \ eye(2);
                    
                    meas = obj.measurement2D;
                end
            end
            
            K = trueCrossCov * invTrueMeasCov;
            
            trueMean = obj.initMean + K * (meas(:) - trueMeasMean);
            trueCov  = obj.initCov  - K * trueCrossCov';
         	
            if analytic
                tol = sqrt(eps);
                
                % Single iteration
                f = AnalyticKF();
                f.setState(Gaussian(obj.initMean, obj.initCov));
                f.update(measModel, meas);
                
                [mean, cov] = f.getPointEstimate();
                
                obj.verifyEqual(mean, trueMean, 'RelTol', tol);
                obj.verifyEqual(cov, trueCov, 'RelTol', tol);
                
                [measurement, ...
                 measMean, ...
                 measCov, ...
                 crossCov, ...
                 numIterations] = f.getLastUpdateData();
                
                obj.verifyEqual(measurement, meas(:), 'RelTol', tol);
                obj.verifyEqual(measMean, trueMeasMean, 'RelTol', tol);
                obj.verifyEqual(measCov, trueMeasCov, 'RelTol', tol);
                obj.verifyEqual(crossCov, trueCrossCov, 'RelTol', tol);
                
                obj.verifyEqual(numIterations, 1);
                
                % Multiple iterations
                f = AnalyticKF();
                f.setMaxNumIterations(3);
                f.setState(Gaussian(obj.initMean, obj.initCov));
                f.update(measModel, meas);
                
                [mean, cov] = f.getPointEstimate();
                
                obj.verifyEqual(mean, trueMean, 'RelTol', tol);
                obj.verifyEqual(cov, trueCov, 'RelTol', tol);
                
                [measurement, ...
                 measMean, ...
                 measCov, ...
                 crossCov, ...
                 numIterations] = f.getLastUpdateData();
                
                obj.verifyEqual(measurement, meas(:), 'RelTol', tol);
                obj.verifyEqual(measMean, trueMeasMean, 'RelTol', tol);
                obj.verifyEqual(measCov, trueMeasCov, 'RelTol', tol);
                obj.verifyEqual(crossCov, trueCrossCov, 'RelTol', tol);
                
                obj.verifyEqual(numIterations, 3);
            else
                tol = 1e-2;
                
                f = SIRPF();
                f.setNumParticles(5000000);
                f.setState(Gaussian(obj.initMean, obj.initCov));
                f.update(measModel, meas);
                
                [mean, cov] = f.getPointEstimate();
                
                obj.verifyEqual(mean, trueMean, 'RelTol', tol);
                obj.verifyEqual(cov, trueCov, 'RelTol', tol);
            end
        end
    end
    
    properties (Constant)
        initMean      = [0.3 -pi]';
        initCov       = [0.5 0.1
                         0.1 3.0];
        measMatrix    = [3    -4
                         pi/4  0
                         0.5   2];
        measNoise2D   = Gaussian([2 -1]', [ 2.0 -0.5
                                           -0.5  1.3]);
        measNoise3D   = Gaussian([2 -1 3]', [ 2.0 -0.5 0.2
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
