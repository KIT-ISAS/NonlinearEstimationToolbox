
classdef TestEnKF < matlab.unittest.TestCase & TestCopy
    % Provides unit tests for the EnKF class.
    
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
            
            obj.verifyEqual(f.getName(), 'EnKF');
            obj.verifyEqual(f.getEnsembleSize(), 1000);
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
            
            obj.checkState(f, 3, 1000);
        end
        
        
        function testSetEnsembleSize(obj)
            f = obj.initFilter();
            
            f.setEnsembleSize(2000);
            
            obj.verifyEqual(f.getEnsembleSize(), 2000);
        end
        
        function testSetEnsembleSizeWithResampling(obj)
            f = obj.initFilter();
            
            d = Gaussian(zeros(2, 1), diag([1.5, 2]));
            f.setState(d);
            
            f.setEnsembleSize(2000);
            
            obj.verifyEqual(f.getEnsembleSize(), 2000);
            
            obj.checkState(f, 2, 2000);
        end
        
        
        function testGetStateMeanAndCov(obj)
            f = obj.initFilter();
            
            f.setEnsembleSize(1e7);
            
            d = Gaussian(zeros(2, 1), diag([1.5, 2]));
            
            f.setState(d);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            [mean, cov] = d.getMeanAndCov();
            
            obj.verifyEqual(stateMean, mean, 'AbsTol', 1e-1);
            obj.verifyEqual(stateCov, cov, 'AbsTol', 1e-1);
        end
        
        
        function testPredictLinearSysModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setEnsembleSize(1000000);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictAddNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setEnsembleSize(1000000);
            
            TestUtilsAdditiveNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setEnsembleSize(1000000);
            
            TestUtilsSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictMixedNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = 1e-2;
            
            f.setEnsembleSize(1000000);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setEnsembleSize(5000000);
            
            testUtils = TestUtilsLinearMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
        end
        
        
        function testUpdateAddNoiseMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setEnsembleSize(5000000);
            
            testUtils = TestUtilsAdditiveNoiseMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
        end
        
        
        function testUpdateMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setEnsembleSize(5000000);
            
            testUtils = TestUtilsMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
        end
        
        
        function testUpdateMixedNoiseMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setEnsembleSize(5000000);
            
            testUtils = TestUtilsMixedNoiseMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = EnKF();
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
