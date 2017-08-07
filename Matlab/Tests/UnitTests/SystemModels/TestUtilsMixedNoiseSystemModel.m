
classdef TestUtilsMixedNoiseSystemModel
    % Provides test utilities for the MixedNoiseSystemModel class.
    
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
            [initState, sysModel, ...
             trueStateMean, trueStateCov] = TestUtilsMixedNoiseSystemModel.getSysModelData();
            
            [f, tol] = setupPrediction();
            
            f.setState(initState);
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            test.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            test.verifyEqual(stateCov, stateCov');
            test.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
        
        function [initState, sysModel, ...
                  trueStateMean, trueStateCov] = getSysModelData()
            initState = Gaussian(TestUtilsMixedNoiseSystemModel.initMean, ...
                                 TestUtilsMixedNoiseSystemModel.initCov);
            
            sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(TestUtilsMixedNoiseSystemModel.addSysNoise);
            sysModel.setNoise(TestUtilsMixedNoiseSystemModel.sysNoise);
            
            mat                         = sysModel.sysMatrix;
            [addNoiseMean, addNoiseCov] = TestUtilsMixedNoiseSystemModel.addSysNoise.getMeanAndCov();
            [noiseMean, noiseCov]       = TestUtilsMixedNoiseSystemModel.sysNoise.getMeanAndCov();
            
            trueStateMean = mat * TestUtilsMixedNoiseSystemModel.initMean       + addNoiseMean + noiseMean;
            trueStateCov  = mat * TestUtilsMixedNoiseSystemModel.initCov * mat' + addNoiseCov  + noiseCov;
        end
    end
    
    properties (Constant, Access = 'private')
        initMean    = [0.3 -pi]';
        initCov     = [0.5 0.1; 0.1 3];
        addSysNoise = Gaussian([2 -1]', 0.01 * [2 -0.5; -0.5 1.3]);
        sysNoise    = Gaussian([0, 3]', 0.1 * diag([0.5, 0.1]));
    end
end
