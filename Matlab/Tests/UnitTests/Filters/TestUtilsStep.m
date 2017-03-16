
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
        function checkAdditiveNoiseSystemModel(test, filter, tol)
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predAddNoiseSysModel();
            
            % True update
            [measModel, measurements, ...
             trueMean, trueCov] = TestUtilsStep.update(truePredMean, truePredCov);
            
            % Test step method
            TestUtilsStep.checkStep(sysModel, measModel, measurements, ...
                                    trueMean, trueCov, test, filter, tol);
        end
        
        function checkAdditiveNoiseSystemModelMultiMeas(test, filter, tol)
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predAddNoiseSysModel();
            
            % True update
            [measModel, measurements, ...
             trueMean, trueCov] = TestUtilsStep.updateMultiMeas(truePredMean, truePredCov);
            
            % Test step method
            TestUtilsStep.checkStep(sysModel, measModel, measurements, ...
                                    trueMean, trueCov, test, filter, tol);
        end
        
        
        function checkSystemModel(test, filter, tol)
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predSysModel();
            
            % True update
            [measModel, measurements, ...
             trueMean, trueCov] = TestUtilsStep.update(truePredMean, truePredCov);
            
            % Test step method
            TestUtilsStep.checkStep(sysModel, measModel, measurements, ...
                                    trueMean, trueCov, test, filter, tol);
        end
        
        function checkSystemModelMultiMeas(test, filter, tol)
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predSysModel();
            
            % True update
            [measModel, measurements, ...
             trueMean, trueCov] = TestUtilsStep.updateMultiMeas(truePredMean, truePredCov);
            
            % Test step method
            TestUtilsStep.checkStep(sysModel, measModel, measurements, ...
                                    trueMean, trueCov, test, filter, tol);
        end
        
        
        function checkMixedNoiseSystemModel(test, filter, tol)
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predMixedNoiseSysModel();
            
            % True update
            [measModel, measurements, ...
             trueMean, trueCov] = TestUtilsStep.update(truePredMean, truePredCov);
            
            % Test step method
            TestUtilsStep.checkStep(sysModel, measModel, measurements, ...
                                    trueMean, trueCov, test, filter, tol);
        end
        
        function checkMixedNoiseSystemModelMultiMeas(test, filter, tol)
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predMixedNoiseSysModel();
            
            % True update
            [measModel, measurements, ...
             trueMean, trueCov] = TestUtilsStep.updateMultiMeas(truePredMean, truePredCov);
            
            % Test step method
            TestUtilsStep.checkStep(sysModel, measModel, measurements, ...
                                    trueMean, trueCov, test, filter, tol);
        end
    end
    
    methods (Static, Access = 'private')
        function [sysModel, truePredMean, truePredCov] = predAddNoiseSysModel()
            sysModel = AddNoiseSysModel();
            sysModel.setNoise(TestUtilsStep.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsStep.initMean + noiseMean;
            truePredCov  = mat * TestUtilsStep.initCov * mat' + noiseCov;
        end
        
        function [sysModel, truePredMean, truePredCov] = predSysModel()
            sysModel = SysModel();
            sysModel.setNoise(TestUtilsStep.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.sysNoise.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsStep.initMean + noiseMean;
            truePredCov  = mat * TestUtilsStep.initCov * mat' + noiseCov;
        end
        
        function [sysModel, truePredMean, truePredCov] = predMixedNoiseSysModel()
            sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(TestUtilsStep.sysNoise);
            sysModel.setNoise(TestUtilsStep.sysNoise2);
            
            mat                         = sysModel.sysMatrix;
            [addNoiseMean, addNoiseCov] = TestUtilsStep.sysNoise.getMeanAndCovariance();
            [noiseMean, noiseCov]       = TestUtilsStep.sysNoise2.getMeanAndCovariance();
            
            truePredMean = mat * TestUtilsStep.initMean       + addNoiseMean + noiseMean;
            truePredCov  = mat * TestUtilsStep.initCov * mat' + addNoiseCov  + noiseCov;
        end
        
        function [measModel, measurements, ...
                  trueMean, trueCov] = update(truePredMean, truePredCov)
            measModel = AddNoiseMeasModel();
            measModel.setNoise(TestUtilsStep.measNoise);
            
            mat                   = measModel.measMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.measNoise.getMeanAndCovariance();
            
            measurements = TestUtilsStep.singleMeas;
            
            trueMeasMean = mat * truePredMean + noiseMean;
            trueMeasCov  = mat * truePredCov * mat' + noiseCov;
            trueCrossCov = truePredCov * mat';
            
            invMeasCov = trueMeasCov \ eye(3);
            
            K = trueCrossCov * invMeasCov;
            
            trueMean = truePredMean + K * (measurements - trueMeasMean);
            trueCov  = truePredCov - K * trueCrossCov';
        end
        
        function [measModel, measurements, ...
                  trueMean, trueCov] = updateMultiMeas(truePredMean, truePredCov)
            measModel = AddNoiseMeasModel();
            measModel.setNoise(TestUtilsStep.measNoise);
            
            mat                   = measModel.measMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.measNoise.getMeanAndCovariance();
            
            measurements = TestUtilsStep.twoMeas;
            
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
        
        function checkStep(sysModel, measModel, measurements, trueMean, trueCov, test, filter, tol)
            filter.setState(Gaussian(TestUtilsStep.initMean, ...
                                     TestUtilsStep.initCov));
            
            filter.step(sysModel, measModel, measurements);
            
            [mean, cov] = filter.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, cov');
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
    end
    
    properties (Constant, Access = 'private')
        initMean   = [0.3 -pi]';
        initCov    = [0.5 0.1; 0.1 3];
        sysNoise   = Gaussian([2 -1]', 0.01 * [ 2   -0.5
                                               -0.5  1.3]);
        sysNoise2  = Gaussian([0, 3]', 0.01 * diag([5, 1]));
        measNoise  = Gaussian([2 -1 0.5]', 10 * [ 2   -0.5 0
                                            -0.5  1.3 0.5
                                             0    0.5 sqrt(2)]);
        singleMeas = [ 1
                      -2
                       5];
        twoMeas    = [ 1  1.5
                      -2 -1.85
                       2 -4   ];
    end
end
