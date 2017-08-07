
classdef TestUtilsStep
    % Provides test utilities for the Filter.step() method.
    
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
        function [initState, sysModel, ...
                  measModel, measurement, ...
                  trueStateMean, trueStateCov] = getAdditiveNoiseSystemModelData()
            initState = Gaussian(TestUtilsStep.initMean, ...
                                 TestUtilsStep.initCov);
            
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predAddNoiseSysModel();
            
            % True update
            [measModel, measurement, ...
             trueStateMean, trueStateCov] = TestUtilsStep.update(truePredMean, truePredCov);
        end
        
        function [initState, sysModel, ...
                  measModel, measurement, ...
                  trueStateMean, trueStateCov] = getSystemModelData()
            initState = Gaussian(TestUtilsStep.initMean, ...
                                 TestUtilsStep.initCov);
            
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predSysModel();
            
            % True update
            [measModel, measurement, ...
             trueStateMean, trueStateCov] = TestUtilsStep.update(truePredMean, truePredCov);
        end
        
        function [initState, sysModel, ...
                  measModel, measurement, ...
                  trueStateMean, trueStateCov] = getMixedNoiseSystemModelData()
            initState = Gaussian(TestUtilsStep.initMean, ...
                                 TestUtilsStep.initCov);
            
            % True prediction
            [sysModel, truePredMean, truePredCov] = TestUtilsStep.predMixedNoiseSysModel();
            
            % True update
            [measModel, measurement, ...
             trueStateMean, trueStateCov] = TestUtilsStep.update(truePredMean, truePredCov);
        end
    end
    
    methods (Static, Access = 'private')
        function [sysModel, truePredMean, truePredCov] = predAddNoiseSysModel()
            sysModel = AddNoiseSysModel();
            sysModel.setNoise(TestUtilsStep.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.sysNoise.getMeanAndCov();
            
            truePredMean = mat * TestUtilsStep.initMean + noiseMean;
            truePredCov  = mat * TestUtilsStep.initCov * mat' + noiseCov;
        end
        
        function [sysModel, truePredMean, truePredCov] = predSysModel()
            sysModel = SysModel();
            sysModel.setNoise(TestUtilsStep.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.sysNoise.getMeanAndCov();
            
            truePredMean = mat * TestUtilsStep.initMean + noiseMean;
            truePredCov  = mat * TestUtilsStep.initCov * mat' + noiseCov;
        end
        
        function [sysModel, truePredMean, truePredCov] = predMixedNoiseSysModel()
            sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(TestUtilsStep.sysNoise);
            sysModel.setNoise(TestUtilsStep.sysNoise2);
            
            mat                         = sysModel.sysMatrix;
            [addNoiseMean, addNoiseCov] = TestUtilsStep.sysNoise.getMeanAndCov();
            [noiseMean, noiseCov]       = TestUtilsStep.sysNoise2.getMeanAndCov();
            
            truePredMean = mat * TestUtilsStep.initMean       + addNoiseMean + noiseMean;
            truePredCov  = mat * TestUtilsStep.initCov * mat' + addNoiseCov  + noiseCov;
        end
        
        function [measModel, measurement, ...
                  trueStateMean, trueStateCov] = update(truePredMean, truePredCov)
            measModel = AddNoiseMeasModel();
            measModel.setNoise(TestUtilsStep.measNoise);
            
            mat                   = measModel.measMatrix;
            [noiseMean, noiseCov] = TestUtilsStep.measNoise.getMeanAndCov();
            
            measurement = TestUtilsStep.meas;
            
            trueMeasMean = mat * truePredMean + noiseMean;
            trueMeasCov  = mat * truePredCov * mat' + noiseCov;
            trueCrossCov = truePredCov * mat';
            
            invMeasCov = trueMeasCov \ eye(3);
            
            K = trueCrossCov * invMeasCov;
            
            trueStateMean = truePredMean + K * (measurement - trueMeasMean);
            trueStateCov  = truePredCov - K * trueCrossCov';
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
        meas       = [1, -2, 5]';
    end
end
