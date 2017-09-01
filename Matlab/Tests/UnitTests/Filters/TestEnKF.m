
classdef TestEnKF < TestFilter
    % Provides unit tests for the EnKF class.
    
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
    
    methods (Test)
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
        
        
        function testSetStateMeanAndCov(obj)
            f = obj.initFilter();
            
            mean    = [-0.75, 4]';
            cov     = diag([0.25 4] / 12);
            covSqrt = sqrt(cov);
            
            f.setStateMeanAndCov(mean, cov, covSqrt);
            
            obj.checkState(f, 2, 1000);
        end
        
        function testSetStateMeanAndCovNoCovSqrt(obj)
            f = obj.initFilter();
            
            mean = [-0.75, 4]';
            cov  = diag([0.25 4] / 12);
            
            f.setStateMeanAndCov(mean, cov);
            
            obj.checkState(f, 2, 1000);
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
            
            [stateMean, stateCov, stateCovSqrt] = f.getStateMeanAndCov();
            
            [mean, cov, covSqrt] = d.getMeanAndCov();
            
            obj.verifyEqual(stateMean, mean, 'AbsTol', 1e-1);
            obj.verifyEqual(stateCov, cov, 'AbsTol', 1e-1);
            obj.verifyEqual(stateCovSqrt, covSqrt, 'AbsTol', 1e-1);
        end
        
        
        function testPredictLinearSystemModel(obj)
            configs    = logical(dec2bin(0:7) - '0');
            numConfigs = 8;
            
            for i = 1:numConfigs
                obj.testPredictLinearSystemModelConfiguration(configs(i, :));
            end
        end
        
        function testPredictAdditiveNoiseSystemModel(obj)
            getSysModelData = @() TestUtilsAdditiveNoiseSystemModel.getSysModelData;
            
            obj.testPredictNonlinear(getSysModelData);
        end
        
        function testPredictSystemModel(obj)
            getSysModelData = @() TestUtilsSystemModel.getSysModelData;
            
            obj.testPredictNonlinear(getSysModelData);
        end
        
        function testPredictMixedNoiseSystemModel(obj)
            getSysModelData = @() TestUtilsMixedNoiseSystemModel.getSysModelData;
            
            obj.testPredictNonlinear(getSysModelData);
        end
        
        
        function testUpdateLinearMeasurementModel(obj)
            % Without measurement matrix set
            obj.testUpdateLinearMeasurementModelConfiguration(false);
            
            % With measurement matrix set
            obj.testUpdateLinearMeasurementModelConfiguration(true);
        end
        
        function testUpdateAdditiveNoiseMeasurementModel(obj)
            getMeasModelData = @() TestUtilsAdditiveNoiseMeasurementModel.getMeasModelData;
            
            obj.testUpdateNonlinear(getMeasModelData);
        end
        
        function testUpdateMeasurementModel(obj)
            getMeasModelData = @() TestUtilsMeasurementModel.getMeasModelData;
            
            obj.testUpdateNonlinear(getMeasModelData);
        end
        
        function testUpdateMixedNoiseMeasurementModel(obj)
            getMeasModelData = @() TestUtilsMixedNoiseMeasurementModel.getMeasModelData;
            
            obj.testUpdateNonlinear(getMeasModelData);
        end
        
        
        function testUpdateInvalidMeasurement(obj)
            f = obj.initFilter();
            
            f.setState(Gaussian(0, 1));
            
            measModel = [];
            
            invalidMeas = ones(2, 4);
            obj.verifyError(@() f.update(measModel, invalidMeas), ...
                            'Filter:InvalidMeasurement');
            
            invalidMeas = { 1, 3 };
            obj.verifyError(@() f.update(measModel, invalidMeas), ...
                            'Filter:InvalidMeasurement');
            
            invalidMeas = Gaussian(0, 1);
            obj.verifyError(@() f.update(measModel, invalidMeas), ...
                            'Filter:InvalidMeasurement');
        end
    end
    
    methods (Access = 'protected')
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestFilter(f);
            
            % EnKF-related tests
            obj.verifyEqual(f.getName(), 'EnKF');
            obj.verifyEqual(f.getEnsembleSize(), 1000);
        end
        
        function f = initFilter(~)
            f = EnKF();
        end
        
        function testPredictLinearSystemModelConfiguration(obj, config)
            hasSysMat      = config(1);
            hasInput       = config(2);
            hasSysNoiseMat = config(3);
            
            [initState, sysModel, ...
             trueStateMean, trueStateCov] = TestUtilsLinearSystemModel.getSysModelData(hasSysMat, ...
                                                                                       hasSysNoiseMat, ...
                                                                                       hasInput);
            
            f = obj.initFilter();
            f.setEnsembleSize(1e7);
            
            tol = 1e-2;
            
            f.setState(initState);
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
        
        function testPredictNonlinear(obj, getSysModelData)
            [initState, sysModel, ...
             trueStateMean, trueStateCov] = getSysModelData();
            
            f = obj.initFilter();
            f.setEnsembleSize(1e6);
            
            tol = 1e-2;
            
            f.setState(initState);
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
        
        function testUpdateLinearMeasurementModelConfiguration(obj, hasMeasMatrix)
            [initState, measModel, ...
             measurement, ~, ...
             trueStateMean, trueStateCov] = TestUtilsLinearMeasurementModel.getMeasModelData(hasMeasMatrix);
            
            f = obj.initFilter();
            f.setEnsembleSize(1e7);
            
            tol = 1e-2;
            
            f.setState(initState);
            
            f.update(measModel, measurement);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
        
        function testUpdateNonlinear(obj, getMeasModelData)
            [initState, measModel, ...
             measurement, ~, ...
             trueStateMean, trueStateCov] = getMeasModelData();
            
            f = obj.initFilter();
            f.setEnsembleSize(1e7);
            
            tol = 1e-2;
            
            f.setState(initState);
            
            f.update(measModel, measurement);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
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
