
classdef TestUtilsLinearSystemModel
    % Provides test utilities for the LinearSystemModel class.
    
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
        function checkPrediction(test, filter, tol)
            configs    = logical(dec2bin(0:7) - '0');
            numConfigs = 8;
            
            for i = 1:numConfigs
                TestUtilsLinearSystemModel.checkPredictionConfig(configs(i, :), test, filter, tol);
            end
        end
    end
    
    methods (Static, Access = 'private')
        function checkPredictionConfig(config, test, f, tol)
            sysMat   = config(1);
            input    = config(2);
            noiseMat = config(3);
            
            sysModel = LinearSystemModel();
            sysModel.setNoise(TestUtilsLinearSystemModel.sysNoise);
            
            [noiseMean, noiseCov] = TestUtilsLinearSystemModel.sysNoise.getMeanAndCov();
            
            if sysMat
                sysModel.setSystemMatrix(TestUtilsLinearSystemModel.sysMatrix);
                
                trueMean = TestUtilsLinearSystemModel.sysMatrix * TestUtilsLinearSystemModel.initMean;
                trueCov  = TestUtilsLinearSystemModel.sysMatrix * TestUtilsLinearSystemModel.initCov * TestUtilsLinearSystemModel.sysMatrix';
            else
                trueMean = TestUtilsLinearSystemModel.initMean;
                trueCov  = TestUtilsLinearSystemModel.initCov;
            end
            
            if input
                sysModel.setSystemInput(TestUtilsLinearSystemModel.sysInput);
                
                trueMean = trueMean + TestUtilsLinearSystemModel.sysInput;
            end
            
            if noiseMat
                sysModel.setSystemNoiseMatrix(TestUtilsLinearSystemModel.sysNoiseMatrix);
                
                trueMean = trueMean + TestUtilsLinearSystemModel.sysNoiseMatrix * noiseMean;
                trueCov  = trueCov + TestUtilsLinearSystemModel.sysNoiseMatrix * noiseCov * TestUtilsLinearSystemModel.sysNoiseMatrix';
            else
                trueMean = trueMean + noiseMean;
                trueCov  = trueCov + noiseCov;
            end
            
            f.setState(Gaussian(TestUtilsLinearSystemModel.initMean, ...
                                TestUtilsLinearSystemModel.initCov));
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            test.verifyEqual(stateMean, trueMean, 'RelTol', tol);
            test.verifyEqual(stateCov, stateCov');
            test.verifyEqual(stateCov, trueCov, 'RelTol', tol);
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
