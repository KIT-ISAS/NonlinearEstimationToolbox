
classdef TestGaussianFilter < TestFilter
    % Provides unit tests for the GaussianFilter class.
    
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
            
            mean = zeros(2, 1);
            cov  = diag([1.5, 2]);
            
            d = Gaussian(mean, cov);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateGaussianMixture(obj)
            f = obj.initFilter();
            
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.31;
            w2 = 0.967;
            
            means   = [m1 m2];
            covs    = cat(3, c1, c2);
            weights = [w1 w2];
            
            d = GaussianMixture(means, covs, weights);
            
            weights  = weights / (w1 + w2);
            w1       = weights(1);
            w2       = weights(2);
            mean     = w1 * m1 + w2 * m2;
            cov      = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - mean) * (m1 - mean)' + ...
                       w2 * (m2 - mean) * (m2 - mean)';
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateUniform(obj)
            f = obj.initFilter();
            
            a = [-1, 3]';
            b = [-0.5, 5]';
            d = Uniform(a, b);
            
            mean = [-0.75, 4]';
            cov  = diag([0.25 4] / 12);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateDiracMixture(obj)
            f = obj.initFilter();
            
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1];
            
            d = DiracMixture(samples, weights);
            
            mean = [-4 -4 -4]';
            cov  = eye(3);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 3);
        end
        
        
        function testSetStateMeanAndCov(obj)
            f = obj.initFilter();
            
            mean    = [-0.75, 4]';
            cov     = diag([0.25 4] / 12);
            covSqrt = sqrt(cov);
            
            f.setStateMeanAndCov(mean, cov, covSqrt);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateMeanAndCovNoCovSqrt(obj)
            f = obj.initFilter();
            
            mean = [-0.75, 4]';
            cov  = diag([0.25 4] / 12);
            
            f.setStateMeanAndCov(mean, cov);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        
        function testGetStateMeanAndCov(obj)
            f = obj.initFilter();
            
            a = [-1, 3]';
            b = [-0.5, 5]';
            d = Uniform(a, b);
            
            mean    = [-0.75, 4]';
            cov     = diag([0.25 4] / 12);
            covSqrt = sqrt(cov);
            
            f.setState(d);
            
            [stateMean, stateCov, stateCovSqrt] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, mean);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, cov);
            obj.verifyEqual(stateCovSqrt, covSqrt);
        end
        
        
        function testSetPredictionPostProcessing(obj)
            f = obj.initFilter();
            
            func = @TestGaussianFilter.postProcessingScale;
            
            f.setPredictionPostProcessing(func);
            
            obj.verifyEqual(f.getPredictionPostProcessing(), func);
        end
        
        
        function testSetUpdatePostProcessing(obj)
            f = obj.initFilter();
            
            func = @TestGaussianFilter.postProcessingScale;
            
            f.setUpdatePostProcessing(func);
            
            obj.verifyEqual(f.getUpdatePostProcessing(), func);
        end
        
        
        function testPredictLinearSystemModel(obj)
            configs    = logical(dec2bin(0:15) - '0');
            numConfigs = 16;
            
            for i = 1:numConfigs
                obj.testPredictLinearSystemModelConfiguration(configs(i, :));
            end
        end
        
        function testPredictAdditiveNoiseSystemModel(obj)
            getSysModelData = @() TestUtilsAdditiveNoiseSystemModel.getSysModelData;
            
            obj.testPredictNonlinearConfigruations(getSysModelData);
        end
        
        function testPredictSystemModel(obj)
            getSysModelData = @() TestUtilsSystemModel.getSysModelData;
            
            obj.testPredictNonlinearConfigruations(getSysModelData);
        end
        
        function testPredictMixedNoiseSystemModel(obj)
            getSysModelData = @() TestUtilsMixedNoiseSystemModel.getSysModelData;
            
            obj.testPredictNonlinearConfigruations(getSysModelData);
        end
        
        
        function testInvalidStateDecompDim(obj)
            f = obj.initFilter();
            
            f.setState(Gaussian(zeros(3, 1), 2 * eye(3)));
            
            f.setStateDecompDim(3);
            
            measModel   = [];
            measurement = [];
            
            obj.verifyError(@() f.update(measModel, measurement), ...
                            'Filter:InvalidUnobservableStateDimension');
        end
    end
    
    methods (Static)
        function [postMean, postCov] = postProcessingScale(mean, cov, ~)
            postMean = 2.0 * mean;
            postCov  = 1.5 * cov;
        end
    end
    
    methods (Access = 'protected')
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestFilter(f);
            
            % GaussianFilter-related tests
            obj.verifyEqual(f.getStateDecompDim(), 0);
            obj.verifyEmpty(f.getPredictionPostProcessing());
            obj.verifyEmpty(f.getUpdatePostProcessing());
        end
        
        function testPredictLinearSystemModelConfiguration(obj, config)
            hasSysMat      = config(1);
            hasInput       = config(2);
            hasSysNoiseMat = config(3);
            postProcessing = config(4);
            
            [initState, sysModel, ...
             trueStateMean, trueStateCov] = TestUtilsLinearSystemModel.getSysModelData(hasSysMat, ...
                                                                                       hasSysNoiseMat, ...
                                                                                       hasInput);
            
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setState(initState);
            
            if postProcessing
                f.setPredictionPostProcessing(@TestGaussianFilter.postProcessingScale);
            end
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            if postProcessing
                [trueStateMean, ...
                 trueStateCov] = TestGaussianFilter.postProcessingScale(trueStateMean, trueStateCov);
            end
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
        
        function testPredictNonlinearConfigruations(obj, getSysModelData)
            % Without state decompoosition
            obj.testPredictNonlinearConfiguration(false, getSysModelData);
            
            % With state decomposition
            obj.testPredictNonlinearConfiguration(true, getSysModelData);
        end
        
        function testPredictNonlinearConfiguration(obj, postProcessing, getSysModelData)
            [initState, sysModel, ...
             trueStateMean, trueStateCov] = getSysModelData();
            
            [f, tol] = obj.setupNonlinarPrediction();
            
            f.setState(initState);
            
            if postProcessing
                f.setPredictionPostProcessing(@TestGaussianFilter.postProcessingScale);
            end
            
            f.predict(sysModel);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            if postProcessing
                [trueStateMean, ...
                 trueStateCov] = TestGaussianFilter.postProcessingScale(trueStateMean, trueStateCov);
            end
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
    end
    
    methods (Abstract, Access = 'protected')
        [f, tol] = setupNonlinarPrediction(obj);
    end
    
    methods (Access = 'private')
        function checkState(obj, f, mean, cov, dim)
            state = f.getState();
            
            obj.verifyClass(state, 'Gaussian');
            
            [stateMean, stateCov] = state.getMeanAndCov();
            
            obj.verifyEqual(stateMean, mean, 'Abstol', 1e-8);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, cov, 'Abstol', 1e-8);
            
            obj.verifyEqual(f.getStateDim(), dim);
        end
    end
end
