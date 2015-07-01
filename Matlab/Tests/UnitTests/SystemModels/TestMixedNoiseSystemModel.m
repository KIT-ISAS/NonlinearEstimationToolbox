
classdef TestMixedNoiseSystemModel < matlab.unittest.TestCase
    % Provides unit tests for the MixedNoiseSystemModel class.
    
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
        function testSimulate(obj)
        	sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(Uniform([0 0], [1 1]));
            sysModel.setNoise(Uniform([0 0], [1 1]));
            
            detSimState = sysModel.sysMatrix * obj.initMean;
            
            simState = sysModel.simulate(obj.initMean);
            
            obj.verifyEqual(size(simState), [2 1]);
            obj.verifyGreaterThanOrEqual(simState, detSimState);
            obj.verifyLessThanOrEqual(simState, detSimState + 2);
        end
        
        function testEKF(obj)
            f = EKF();
            
            obj.checkPrediction(f);
        end
        
        function testCKF(obj)
            f = CKF();
            
            obj.checkPrediction(f);
        end
        
        function testGHKF(obj)
            f = GHKF();
            
            obj.checkPrediction(f);
        end
        
        function testRUKF(obj)
            f = RUKF();
            
            obj.checkPrediction(f);
        end
        
        function testS2KF(obj)
            f = S2KF();
            
            obj.checkPrediction(f);
        end
        
        function testUKF(obj)
            f = UKF();
            
            obj.checkPrediction(f);
        end
        
        function PGF(obj)
            f = PGF();
            
            obj.checkPrediction(f);
        end
        
        function testEnKF(obj)
            f = EnKF();
            f.setEnsembleSize(1000000);
            
            obj.checkPrediction(f, 1e-2);
        end
        
        function testGPF(obj)
            f = GPF();
            f.setNumParticles(1000000);
            
            obj.checkPrediction(f, 1e-2);
        end
        
        function testSIRPF(obj)
            f = SIRPF();
            f.setNumParticles(1000000);
            
            obj.checkPrediction(f, 1e-2);
        end
    end
    
    methods (Access = 'private')
        function checkPrediction(obj, f, tol)
            if nargin < 3
                tol = sqrt(eps);
            end
            
        	sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(obj.addSysNoise);
            sysModel.setNoise(obj.sysNoise);
            
            mat                         = sysModel.sysMatrix;
            [addNoiseMean, addNoiseCov] = obj.addSysNoise.getMeanAndCovariance();
            [noiseMean, noiseCov]       = obj.sysNoise.getMeanAndCovariance();
            
            trueMean = mat * obj.initMean       + addNoiseMean + noiseMean;
            trueCov  = mat * obj.initCov * mat' + addNoiseCov  + noiseCov;
            
            f.setState(Gaussian(obj.initMean, obj.initCov));
            
            f.predict(sysModel);
            
            [mean, cov] = f.getPointEstimate();
            
            obj.verifyEqual(mean, trueMean, 'RelTol', tol);
            obj.verifyEqual(cov, trueCov, 'RelTol', tol);
        end
    end
    
    properties (Constant)
        initMean    = [0.3 -pi]';
        initCov     = [0.5 0.1; 0.1 3];
        addSysNoise = Gaussian([2 -1]', [2 -0.5; -0.5 1.3]);
        sysNoise    = Gaussian([-5, 3]', diag([0.98, 2.1]));
    end
end
