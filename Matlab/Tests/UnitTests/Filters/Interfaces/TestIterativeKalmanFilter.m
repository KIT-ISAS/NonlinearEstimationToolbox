
classdef TestIterativeKalmanFilter < TestLinearGaussianFilter
    % Provides unit tests for the IterativeKalmanFilter class.
    
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
        function testSetMaxNumIterations(obj)
            f = obj.initFilter();
            
            f.setMaxNumIterations(5);
            
            obj.verifyEqual(f.getMaxNumIterations(),5);
        end
        
        
        function testSetConvergenceCheck(obj)
            f = obj.initFilter();
            
            func = @TestIterativeKalmanFilter.convergenceCheckTrue;
            
            f.setConvergenceCheck(func);
            
            obj.verifyEqual(f.getConvergenceCheck(), func);
        end
        
        
        function testUpdateAdditiveNoiseMeasurementModel(obj)
            getMeasModelData = @(stateDecomp) TestUtilsAdditiveNoiseMeasurementModel.getMeasModelData(stateDecomp);
            
            obj.testUpdateNonlinearConfigruations(getMeasModelData);
        end
        
        function testUpdateMeasurementModel(obj)
            getMeasModelData = @(stateDecomp) TestUtilsMeasurementModel.getMeasModelData(stateDecomp);
            
            obj.testUpdateNonlinearConfigruations(getMeasModelData);
        end
        
        function testUpdateMixedNoiseMeasurementModel(obj)
            getMeasModelData = @(stateDecomp) TestUtilsMixedNoiseMeasurementModel.getMeasModelData(stateDecomp);
            
            obj.testUpdateNonlinearConfigruations(getMeasModelData);
        end
    end
    
    methods (Access = 'protected')
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestLinearGaussianFilter(f);
            
            % IterativeKalmanFilter-related tests
            obj.verifyEqual(f.getMaxNumIterations(), 1);
            obj.verifyEqual(f.getNumIterations(), 0);
            obj.verifyEmpty(f.getConvergenceCheck());
        end
        
        function testUpdateNonlinearConfigruations(obj, getMeasModelData)
            configs    = logical(dec2bin(0:31) - '0');
            numConfigs = 32;
            
            for i = 1:numConfigs
                obj.testUpdateNonlinearConfiguration(configs(i, :), getMeasModelData);
            end
        end
        
        function testUpdateNonlinearConfiguration(obj, config, getMeasModelData)
            stateDecomp      = config(1);
            postProcessing   = config(2);
            measGating       = config(3);
            multiIterations  = config(4);
            convergenceCheck = config(5);
            
            [initState, measModel, ...
             measurement, stateDecompDim, ...
             trueStateMean, trueStateCov, ...
             trueMeasMean, trueMeasCov] = getMeasModelData(stateDecomp);
            
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setState(initState);
            
            if stateDecomp
                f.setStateDecompDim(stateDecompDim);
            end
            
            if postProcessing
                f.setUpdatePostProcessing(@TestGaussianFilter.postProcessingScale);
            end
            
            if multiIterations
                numIterations = 3;
                f.setMaxNumIterations(numIterations);
            else
                numIterations = 1;
            end
            
            if convergenceCheck
                f.setConvergenceCheck(@TestIterativeKalmanFilter.convergenceCheckTrue);
            end
            
            if measGating
                % Compute threshold that should fail
                dimMeas  = size(trueMeasMean, 1);
                measDist = (trueMeasMean - measurement)' * trueMeasCov^(-1) * (trueMeasMean - measurement);
                
                normalizedDist = chi2cdf(measDist, dimMeas);
                
                f.setMeasGatingThreshold(normalizedDist * 0.5);
                
                obj.verifyWarning(@() f.update(measModel, measurement), ...
                                  'Filter:IgnoringMeasurement');
                
                [stateMean, stateCov] = f.getStateMeanAndCov();
                
                [initStateMean, initStateCov] = initState.getMeanAndCov();
                
                % State estimate should not change
                obj.verifyEqual(stateMean, initStateMean);
                obj.verifyEqual(stateCov, stateCov');
                obj.verifyEqual(stateCov, initStateCov);
                obj.verifyEqual(f.getNumIterations(), 1);
            else
                f.update(measModel, measurement);
                
                [stateMean, stateCov] = f.getStateMeanAndCov();
                
                if postProcessing
                    [trueStateMean, ...
                     trueStateCov] = TestGaussianFilter.postProcessingScale(trueStateMean, trueStateCov);
                end
                
                obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
                obj.verifyEqual(stateCov, stateCov');
                obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
                
                if convergenceCheck
                    obj.verifyEqual(f.getNumIterations(), 1);
                else
                    obj.verifyEqual(f.getNumIterations(), numIterations);
                end
            end
        end
    end
    
    methods (Static, Access = 'protected')
        function convReached = convergenceCheckTrue(~, ~, ~, ~, ~, ~)
            convReached = true;
        end
    end
end
