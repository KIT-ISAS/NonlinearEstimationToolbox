
classdef TestLinearSystemModel < matlab.unittest.TestCase
    % Provides unit tests for the LinearSystemModel class.
    
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
            sysModel = LinearSystemModel();
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testConstructorSysMatrix(obj)
            sysModel = LinearSystemModel(eye(2));
            
            obj.verifyEqual(sysModel.sysMatrix, eye(2));
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testConstructorSysMatrixSysNoiseMatrix(obj)
            sysModel = LinearSystemModel(eye(2), 2 * ones(2, 3));
            
            obj.verifyEqual(sysModel.sysMatrix, eye(2));
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, 2 * ones(2, 3));
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSetSystemMatrix(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemMatrix(eye(3));
            
            obj.verifyEqual(sysModel.sysMatrix, eye(3));
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSetSystemMatrixEmpty(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemMatrix([]);
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSetSystemInput(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemInput([1 2 -3]');
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, [1 2 -3]');
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSetSystemInputEmpty(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemInput([]);
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSetSystemNoiseMatrix(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemNoiseMatrix(ones(3, 2));
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, ones(3, 2));
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSetSystemNoiseMatrixEmpty(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemNoiseMatrix([]);
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSimulate(obj)
            sysModel = LinearSystemModel(obj.sysMatrix);
            sysModel.setNoise(Uniform([0 0], [1 1]));
            
            trueState = [sqrt(2) exp(1)]';
            
            detSimState = obj.sysMatrix * trueState;
            
            simState = sysModel.simulate(trueState);
            
            obj.verifyEqual(size(simState), [2 1]);
            obj.verifyGreaterThanOrEqual(simState, detSimState);
            obj.verifyLessThanOrEqual(simState, detSimState + 1);
        end
        
        
        function testNoSysMatNoInputNoSysNoiseNonAnalytic(obj)
            obj.checkPrediction(false, false, false, false);
        end
        
        function testSysMatNoInputNoSysNoiseNonAnalytic(obj)
            obj.checkPrediction(true, false, false, false);
        end
        
        
        function testNoSysMatInputNoSysNoiseNonAnalytic(obj)
            obj.checkPrediction(false, true, false, false);
        end
        
        function testSysMatInputNoSysNoiseNonAnalytic(obj)
            obj.checkPrediction(true, true, false, false);
        end
        
        
        function testNoSysMatNoInputSysNoiseNonAnalytic(obj)
            obj.checkPrediction(false, false, true, false);
        end
        
        function testSysMatNoInputSysNoiseNonAnalytic(obj)
            obj.checkPrediction(true, false, true, false);
        end
        
        
        function testNoSysMatInputSysNoiseNonAnalytic(obj)
            obj.checkPrediction(false, true, true, false);
        end
        
        function testSysMatInputSysNoiseNonAnalytic(obj)
            obj.checkPrediction(true, true, true, false);
        end
        
        
        
        function testNoSysMatNoInputNoSysNoiseAnalytic(obj)
            obj.checkPrediction(false, false, false, true);
        end
        
        function testSysMatNoInputNoSysNoiseAnalytic(obj)
            obj.checkPrediction(true, false, false, true);
        end
        
        
        function testNoSysMatInputNoSysNoiseAnalytic(obj)
            obj.checkPrediction(false, true, false, true);
        end
        
        function testSysMatInputNoSysNoiseAnalytic(obj)
            obj.checkPrediction(true, true, false, true);
        end
        
        
        function testNoSysMatNoInputSysNoiseAnalytic(obj)
            obj.checkPrediction(false, false, true, true);
        end
        
        function testSysMatNoInputSysNoiseAnalytic(obj)
            obj.checkPrediction(true, false, true, true);
        end
        
        
        function testNoSysMatInputSysNoiseAnalytic(obj)
            obj.checkPrediction(false, true, true, true);
        end
        
        function testSysMatInputSysNoiseAnalytic(obj)
            obj.checkPrediction(true, true, true, true);
        end
    end
    
    methods (Access = 'private')
        function checkPrediction(obj, sysMat, input, noiseMat, analytic)
        	sysModel = LinearSystemModel();
            sysModel.setNoise(obj.sysNoise);
            
            [noiseMean, noiseCov] = obj.sysNoise.getMeanAndCovariance();
            
            if sysMat
                sysModel.setSystemMatrix(obj.sysMatrix);
                
                trueMean = obj.sysMatrix * obj.initMean;
                trueCov  = obj.sysMatrix * obj.initCov * obj.sysMatrix';
            else
               	trueMean = obj.initMean;
                trueCov  = obj.initCov;
            end
            
            if input
                sysModel.setSystemInput(obj.sysInput);
                
                trueMean = trueMean + obj.sysInput;
            end
            
            if noiseMat
                sysModel.setSystemNoiseMatrix(obj.sysNoiseMatrix);
                
                trueMean = trueMean + obj.sysNoiseMatrix * noiseMean;
                trueCov  = trueCov + obj.sysNoiseMatrix * noiseCov * obj.sysNoiseMatrix';
            else
                trueMean = trueMean + noiseMean;
                trueCov  = trueCov + noiseCov;
            end
            
            if analytic
                f = AnalyticKF();
                tol = sqrt(eps);
            else
                f = SIRPF();
                f.setNumParticles(5000000);
                tol = 1e-2;
            end
            
            f.setState(Gaussian(obj.initMean, obj.initCov));
            
            f.predict(sysModel);
            
            [mean, cov] = f.getPointEstimate();
            
            obj.verifyEqual(mean, trueMean, 'RelTol', tol);
            obj.verifyEqual(cov, trueCov, 'RelTol', tol);
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
