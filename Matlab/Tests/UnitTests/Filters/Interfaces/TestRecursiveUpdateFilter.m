
classdef TestRecursiveUpdateFilter < TestLinearGaussianFilter
    % Provides unit tests for the RecursiveUpdateFilter class.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2017  Jannik Steinbring <jannik.steinbring@kit.edu>
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
        function testSetNumIterations(obj)
            f = obj.initFilter();
            
            f.setNumIterations(5);
            
            obj.verifyEqual(f.getNumIterations(),5);
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
            
            % RecursiveUpdateFilter-related tests
            obj.verifyEqual(f.getNumIterations(), 10);
        end
        
        function testUpdateNonlinearConfigruations(obj, getMeasModelData)
            configs    = logical(dec2bin(0:7) - '0');
            numConfigs = 8;
            
            for i = 1:numConfigs
                obj.testUpdateNonlinearConfiguration(configs(i, :), getMeasModelData);
            end
        end
        
        function testUpdateNonlinearConfiguration(obj, config, getMeasModelData)
            stateDecomp    = config(1);
            postProcessing = config(2);
            measGating     = config(3);
            
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
            end
        end
    end
end
