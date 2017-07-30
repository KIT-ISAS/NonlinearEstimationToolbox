
classdef TestSIRPF < matlab.unittest.TestCase & TestCopy
    % Provides unit tests for the SIRPF class.
    
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
        function testConstructorDefault(obj)
            f = obj.initFilter();
            
            obj.verifyEqual(f.getName(), 'SIRPF');
            obj.verifyEqual(f.getNumParticles(), 1000);
            obj.verifyEqual(f.getMinAllowedNormalizedESS(), 0.5);
        end
        
        
        function testSetStateGaussian(obj)
            f = obj.initFilter();
            
            d = Gaussian(zeros(2, 1), diag([1.5, 2]));
            
            f.setState(d);
            
            obj.checkState(f, 2, 1000);
        end
        
        function testSetStateGaussianMixture(obj)
            f = obj.initFilter();
            
            d = GaussianMixture(ones(2, 2), cat(3, diag([1.5, 2]), 3 * eye(2)), [1 5]);
            
            f.setState(d);
            
            obj.checkState(f, 2, 1000);
        end
        
        function testSetStateUniform(obj)
            f = obj.initFilter();
            
            d = Uniform([-2 3 0], [5 10 1]);
            
            f.setState(d);
            
            obj.checkState(f, 3, 1000);
        end
        
        function testSetStateDiracMixture(obj)
            f = obj.initFilter();
            
            s = cat(2, zeros(3, 1), ones(3, 1), [-2 5 0.7]');
            w = [1 2 3];
            
            d = DiracMixture(s, w);
            
            f.setState(d);
            
            dm = f.getState();
            
            obj.verifyClass(dm, 'DiracMixture');
            
            [samples, weights] = dm.getComponents();
            
            obj.verifyEqual(samples, s);
            obj.verifyEqual(weights, w / sum(w));
            obj.verifyEqual(f.getNumParticles(), 3);
        end
        
        
        function testSetNumParticles(obj)
            f = obj.initFilter();
            
            f.setNumParticles(2000);
            
            obj.verifyEqual(f.getNumParticles(), 2000);
        end
        
        function testSetNumParticlesWithResampling(obj)
            f = obj.initFilter();
            
            d = Gaussian(zeros(2, 1), diag([1.5, 2]));
            f.setState(d);
            
            f.setNumParticles(2000);
            
            obj.verifyEqual(f.getNumParticles(), 2000);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [2, 2000]);
            obj.verifySize(weights, [1, 2000]);
            obj.verifyEqual(weights, repmat(1/2000, 1, 2000), 'AbsTol', 1e-14);
        end
        
        
        function testSetMinAllowedNormalizedESS(obj)
            f = obj.initFilter();
            
            f.setMinAllowedNormalizedESS(0.3);
            
            obj.verifyEqual(f.getMinAllowedNormalizedESS(), 0.3);
        end
        
        
        function testGetStateMeanAndCov(obj)
            f = obj.initFilter();
            
            s = cat(2, zeros(3, 1), ones(3, 1), [-2 5 0.7]');
            w = [1 2 3];
            
            d = DiracMixture(s, w);
            
            f.setState(d);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            [mean, cov] = d.getMeanAndCov();
            
            obj.verifyEqual(stateMean, mean);
            obj.verifyEqual(stateCov, cov);
        end
        
        
        function testPredictLinearSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictAddNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsAdditiveNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsSystemModel.checkPrediction(obj,f, tol);
        end
        
        function testPredictMixedNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            testUtils = TestUtilsLinearMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
        end
        
        
        function testUpdateAddNoiseMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            testUtils = TestUtilsAdditiveNoiseMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = SIRPF();
        end
    end
    
    methods (Access = 'private')
        function checkState(obj, f, dim, numSamples)
            dm = f.getState();
            
            obj.verifyClass(dm, 'DiracMixture');
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [dim, numSamples]);
            obj.verifySize(weights, [1, numSamples]);
            obj.verifyEqual(weights, repmat(1/numSamples, 1, numSamples), 'AbsTol', 1e-14);
            
            obj.verifyEqual(f.getStateDim(), dim);
        end
    end
end
