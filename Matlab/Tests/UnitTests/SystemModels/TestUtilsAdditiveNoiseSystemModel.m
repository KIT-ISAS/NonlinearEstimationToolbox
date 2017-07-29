
classdef TestUtilsAdditiveNoiseSystemModel
    % Provides test utilities for the AdditiveNoiseSystemModel class.
    
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
        function checkPrediction(test, f, tol)
            sysModel = AddNoiseSysModel();
            sysModel.setNoise(TestUtilsAdditiveNoiseSystemModel.sysNoise);
            
            mat                   = sysModel.sysMatrix;
            [noiseMean, noiseCov] = TestUtilsAdditiveNoiseSystemModel.sysNoise.getMeanAndCov();
            
            trueMean = mat * TestUtilsAdditiveNoiseSystemModel.initMean + noiseMean;
            trueCov  = mat * TestUtilsAdditiveNoiseSystemModel.initCov * mat' + noiseCov;
            
            f.setState(Gaussian(TestUtilsAdditiveNoiseSystemModel.initMean, ...
                                TestUtilsAdditiveNoiseSystemModel.initCov));
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            test.verifyEqual(stateMean, trueMean, 'RelTol', tol);
            test.verifyEqual(stateCov, stateCov');
            test.verifyEqual(stateCov, trueCov, 'RelTol', tol);
        end
    end
    
    properties (Constant, Access = 'private')
        initMean = [0.3 -pi]';
        initCov  = [0.5 0.1; 0.1 3];
        sysNoise = Gaussian([2 -1]', 0.01 * [2 -0.5; -0.5 1.3]);
    end
end
