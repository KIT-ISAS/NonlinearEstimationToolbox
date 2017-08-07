
classdef TestUtilsLinearSystemModel
    % Provides test utilities for the LinearSystemModel class.
    
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
        function testPrediction(test, setupPrediction)
            configs    = logical(dec2bin(0:7) - '0');
            numConfigs = 8;
            
            for i = 1:numConfigs
                TestUtilsLinearSystemModel.testPredictionConfiguration(configs(i, :), test, setupPrediction);
            end
        end
        
        function [initState, sysModel, ...
                  trueStateMean, trueStateCov] = getSysModelData(hasSysMat, hasSysNoiseMat, hasInput)
            initState = Gaussian(TestUtilsLinearSystemModel.initMean, ...
                                 TestUtilsLinearSystemModel.initCov);
            
            sysModel = LinearSystemModel();
            sysModel.setNoise(TestUtilsLinearSystemModel.sysNoise);
            
            [noiseMean, noiseCov] = TestUtilsLinearSystemModel.sysNoise.getMeanAndCov();
            
            if hasSysMat
                sysModel.setSystemMatrix(TestUtilsLinearSystemModel.sysMatrix);
                
                trueStateMean = TestUtilsLinearSystemModel.sysMatrix * TestUtilsLinearSystemModel.initMean;
                trueStateCov  = TestUtilsLinearSystemModel.sysMatrix * TestUtilsLinearSystemModel.initCov * TestUtilsLinearSystemModel.sysMatrix';
            else
                trueStateMean = TestUtilsLinearSystemModel.initMean;
                trueStateCov  = TestUtilsLinearSystemModel.initCov;
            end
            
            if hasInput
                sysModel.setSystemInput(TestUtilsLinearSystemModel.sysInput);
                
                trueStateMean = trueStateMean + TestUtilsLinearSystemModel.sysInput;
            end
            
            if hasSysNoiseMat
                sysModel.setSystemNoiseMatrix(TestUtilsLinearSystemModel.sysNoiseMatrix);
                
                trueStateMean = trueStateMean + TestUtilsLinearSystemModel.sysNoiseMatrix * noiseMean;
                trueStateCov  = trueStateCov + TestUtilsLinearSystemModel.sysNoiseMatrix * noiseCov * TestUtilsLinearSystemModel.sysNoiseMatrix';
            else
                trueStateMean = trueStateMean + noiseMean;
                trueStateCov  = trueStateCov + noiseCov;
            end
        end
    end
    
    methods (Static, Access = 'private')
        function testPredictionConfiguration(config, test, setupPrediction)
            hasSysMat      = config(1);
            hasInput       = config(2);
            hasSysNoiseMat = config(3);
            
            [initState, sysModel, ...
             trueStateMean, trueStateCov] = TestUtilsLinearSystemModel.getSysModelData(hasSysMat, ...
                                                                                       hasSysNoiseMat, ...
                                                                                       hasInput);
            
            [f, tol] = setupPrediction();
            
            f.setState(initState);
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            test.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            test.verifyEqual(stateCov, stateCov');
            test.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
    end
    
    properties (Constant, Access = 'private')
        initMean       = [0.3 -pi]';
        initCov        = [0.5 0.1; 0.1 3];
        sysMatrix      = [3 -4; 0 2];
        sysInput       = [2.5 -0.1]';
        sysNoiseMatrix = [0.01 2; -1.2 0];
        sysNoise       = Gaussian([2 -1]', [2 -0.5; -0.5 1.3]);
    end
end
