
classdef TestUtilsMixedNoiseSystemModel
    % Provides test utilities for the MixedNoiseSystemModel class.
    
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
            if nargin < 3
                tol = sqrt(eps);
            end
            
            sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(TestUtilsMixedNoiseSystemModel.addSysNoise);
            sysModel.setNoise(TestUtilsMixedNoiseSystemModel.sysNoise);
            
            mat                         = sysModel.sysMatrix;
            [addNoiseMean, addNoiseCov] = TestUtilsMixedNoiseSystemModel.addSysNoise.getMeanAndCovariance();
            [noiseMean, noiseCov]       = TestUtilsMixedNoiseSystemModel.sysNoise.getMeanAndCovariance();
            
            trueMean = mat * TestUtilsMixedNoiseSystemModel.initMean       + addNoiseMean + noiseMean;
            trueCov  = mat * TestUtilsMixedNoiseSystemModel.initCov * mat' + addNoiseCov  + noiseCov;
            
            f.setState(Gaussian(TestUtilsMixedNoiseSystemModel.initMean, ...
                                TestUtilsMixedNoiseSystemModel.initCov));
            
            f.predict(sysModel);
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
    end
    
    properties (Constant)
        initMean    = [0.3 -pi]';
        initCov     = [0.5 0.1; 0.1 3];
        addSysNoise = Gaussian([2 -1]', [2 -0.5; -0.5 1.3]);
        sysNoise    = Gaussian([-5, 3]', diag([0.98, 2.1]));
    end
end
