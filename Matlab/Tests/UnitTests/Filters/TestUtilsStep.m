
classdef TestUtilsStep
    % Provides test utilities for the Filter.step() method.
    
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
        function checkAdditiveNoiseSystemModel(test, f, tol)
            % True prediction
            sysModel = AddNoiseSysModel();
            sysModel.setNoise(TestUtilsAdditiveNoiseSystemModel.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsAdditiveNoiseSystemModel.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsAdditiveNoiseSystemModel.initMean + noiseMean;
            truePredCov  = mat * TestUtilsAdditiveNoiseSystemModel.initCov * mat' + noiseCov;
            
            % True update
            [measModel, measurements, trueMean, trueCov] = TestUtilsStep.performUpdate(truePredMean, truePredCov);
            
            % Test step method
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.step(sysModel, measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        function checkAdditiveNoiseSystemModelMultiMeas(test, f, tol)
            % True prediction
            sysModel = AddNoiseSysModel();
            sysModel.setNoise(TestUtilsAdditiveNoiseSystemModel.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsAdditiveNoiseSystemModel.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsAdditiveNoiseSystemModel.initMean + noiseMean;
            truePredCov  = mat * TestUtilsAdditiveNoiseSystemModel.initCov * mat' + noiseCov;
            
            % True update
            [measModel, measurements, trueMean, trueCov] = TestUtilsStep.performUpdateMultiMeas(truePredMean, truePredCov);
            
            % Test step method
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.step(sysModel, measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        
        function checkSystemModel(test, f, tol)
            % True prediction
            sysModel = SysModel();
            sysModel.setNoise(TestUtilsSystemModel.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsSystemModel.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsSystemModel.initMean + noiseMean;
            truePredCov  = mat * TestUtilsSystemModel.initCov * mat' + noiseCov;
            
            % True update
            [measModel, measurements, trueMean, trueCov] = TestUtilsStep.performUpdate(truePredMean, truePredCov);
            
            % Test step method
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.step(sysModel, measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        function checkSystemModelMultiMeas(test, f, tol)
            % True prediction
            sysModel = SysModel();
            sysModel.setNoise(TestUtilsSystemModel.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsSystemModel.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsSystemModel.initMean + noiseMean;
            truePredCov  = mat * TestUtilsSystemModel.initCov * mat' + noiseCov;
            
            % True update
            [measModel, measurements, trueMean, trueCov] = TestUtilsStep.performUpdateMultiMeas(truePredMean, truePredCov);
            
            % Test step method
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.step(sysModel, measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        
        function checkMixedNoiseSystemModel(test, f, tol)
            % True prediction
            sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(TestUtilsMixedNoiseSystemModel.addSysNoise);
            sysModel.setNoise(TestUtilsMixedNoiseSystemModel.sysNoise);
            
            mat                         = sysModel.sysMatrix;
            [addNoiseMean, addNoiseCov] = TestUtilsMixedNoiseSystemModel.addSysNoise.getMeanAndCovariance();
            [noiseMean, noiseCov]       = TestUtilsMixedNoiseSystemModel.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsMixedNoiseSystemModel.initMean       + addNoiseMean + noiseMean;
            truePredCov  = mat * TestUtilsMixedNoiseSystemModel.initCov * mat' + addNoiseCov  + noiseCov;
            
            % True update
            [measModel, measurements, trueMean, trueCov] = TestUtilsStep.performUpdate(truePredMean, truePredCov);
            
            % Test step method
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.step(sysModel, measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
        
        function checkMixedNoiseSystemModelMultiMeas(test, f, tol)
            % True prediction
            sysModel = SysModel();
            sysModel.setNoise(TestUtilsSystemModel.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsSystemModel.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsSystemModel.initMean + noiseMean;
            truePredCov  = mat * TestUtilsSystemModel.initCov * mat' + noiseCov;
            
            % True update
            [measModel, measurements, trueMean, trueCov] = TestUtilsStep.performUpdateMultiMeas(truePredMean, truePredCov);
            
            % Test step method
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.step(sysModel, measModel, measurements);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
    end
    
    methods (Static, Access = 'private')
        function [measModel, measurements, trueMean, trueCov] = performUpdate(truePredMean, truePredCov)
            measModel = AddNoiseMeasModel();
            measModel.setNoise(TestUtilsAdditiveNoiseMeasurementModel.measNoise);
            
            mat                   = measModel.measMatrix;
            [noiseMean, noiseCov] = TestUtilsAdditiveNoiseMeasurementModel.measNoise.getMeanAndCovariance();
            
            measurements = [1 -2 5]';
            
            trueMeasMean = mat * truePredMean + noiseMean;
            trueMeasCov  = mat * truePredCov * mat' + noiseCov;
            trueCrossCov = truePredCov * mat';
            
            invMeasCov = trueMeasCov \ eye(3);
            
            K = trueCrossCov * invMeasCov;
            
            trueMean = truePredMean + K * (measurements - trueMeasMean);
            trueCov  = truePredCov - K * trueCrossCov';
        end
        
        function [measModel, measurements, trueMean, trueCov] = performUpdateMultiMeas(truePredMean, truePredCov)
            measModel = AddNoiseMeasModel();
            measModel.setNoise(TestUtilsAdditiveNoiseMeasurementModel.measNoise);
            
            mat                   = measModel.measMatrix;
            [noiseMean, noiseCov] = TestUtilsAdditiveNoiseMeasurementModel.measNoise.getMeanAndCovariance();
            
            measurements = [ 1  1.5
                            -2 -1.85
                             2 -4   ];
            
            trueMeasMean = mat * truePredMean + noiseMean;
            trueMeasMean = repmat(trueMeasMean, 2, 1);
            trueMeasCov  = mat * truePredCov * mat';
            trueMeasCov  = [trueMeasCov + noiseCov trueMeasCov
                            trueMeasCov            trueMeasCov + noiseCov];
            trueCrossCov = truePredCov * mat';
            trueCrossCov = [trueCrossCov trueCrossCov];
            
            invMeasCov = trueMeasCov \ eye(6);
            
            K = trueCrossCov * invMeasCov;
            
            trueMean = truePredMean + K * (measurements(:) - trueMeasMean);
            trueCov  = truePredCov - K * trueCrossCov';
        end
    end
end
