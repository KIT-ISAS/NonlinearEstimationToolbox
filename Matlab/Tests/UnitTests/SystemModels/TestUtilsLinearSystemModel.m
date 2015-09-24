
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
        function checkPrediction(test, f, tol)
            TestUtilsLinearSystemModel.testNoSysMatNoInputNoSysNoise(test, f, tol);
            TestUtilsLinearSystemModel.testSysMatNoInputNoSysNoise(test, f, tol);
            
            TestUtilsLinearSystemModel.testNoSysMatInputNoSysNoise(test, f, tol);
            TestUtilsLinearSystemModel.testSysMatInputNoSysNoise(test, f, tol);
            
            TestUtilsLinearSystemModel.testNoSysMatNoInputSysNoise(test, f, tol);
            TestUtilsLinearSystemModel.testSysMatNoInputSysNoise(test, f, tol);
            
            TestUtilsLinearSystemModel.testNoSysMatInputSysNoise(test, f, tol);
            TestUtilsLinearSystemModel.testSysMatInputSysNoise(test, f, tol);
        end
    end
    
    methods (Static, Access = 'private')
        function testNoSysMatNoInputNoSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(false, false, false, test, f, tol);
        end
        
        function testSysMatNoInputNoSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(true, false, false, test, f, tol);
        end
        
        
        function testNoSysMatInputNoSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(false, true, false, test, f, tol);
        end
        
        function testSysMatInputNoSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(true, true, false, test, f, tol);
        end
        
        
        function testNoSysMatNoInputSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(false, false, true, test, f, tol);
        end
        
        function testSysMatNoInputSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(true, false, true, test, f, tol);
        end
        
        
        function testNoSysMatInputSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(false, true, true, test, f, tol);
        end
        
        function testSysMatInputSysNoise(test, f, tol)
            TestUtilsLinearSystemModel.checkPredictionConfig(true, true, true, test, f, tol);
        end
        
        
        function checkPredictionConfig(sysMat, input, noiseMat, test, f, tol)
        	sysModel = LinearSystemModel();
            sysModel.setNoise(TestUtilsLinearSystemModel.sysNoise);
            
            [noiseMean, noiseCov] = TestUtilsLinearSystemModel.sysNoise.getMeanAndCovariance();
            
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
            
            [mean, cov] = f.getPointEstimate();
            
            test.verifyEqual(mean, trueMean, 'RelTol', tol);
            test.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
    end
    
    properties (Constant)
        initMean       = [0.3 -pi]';
        initCov        = [0.5 0.1; 0.1 3];
        sysMatrix      = [3 -4; 0 2];
        sysInput       = [2.5 -0.1]';
        sysNoiseMatrix = [0.01 2; -1.2 0];
        sysNoise       = Gaussian([2 -1]', [2 -0.5; -0.5 1.3]);
    end
end
