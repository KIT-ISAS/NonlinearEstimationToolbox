
classdef TestUtilsMeasurementModels < handle
    % Abstract class for measurement model test utilities
    
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
    
    methods
        function checkUpdate(obj, test, filter, tol)
            obj.checkUpdateConfig(false, false, test, filter, tol);
        end
        
        function checkUpdateStateDecomp(obj, test, filter, tol)
            obj.checkUpdateConfig(true, false, test, filter, tol);
        end
        
        function checkUpdateMultiMeas(obj, test, filter, tol)
            obj.checkUpdateConfig(false, true, test, filter, tol);
        end
        
        function checkUpdateStateDecompMultiMeas(obj, test, filter, tol)
            obj.checkUpdateConfig(true, true, test, filter, tol);
        end
        
        
        function checkUpdateKF(obj, test, createFilter)
            configs    = logical(dec2bin(0:15) - '0');
            numConfigs = 16;
            
            for i = 1:numConfigs
                filter = createFilter();
                
                obj.checkUpdateConfigKF(configs(i, :), test, filter);
            end
        end
    end
    
    methods (Access = 'protected')
        function checkUpdateConfig(obj, stateDecomp, multiMeas, test, filter, tol)
            [initState, measModel, measurements,  ...
             trueStateMean, trueStateCov] = obj.updateConfig(stateDecomp, multiMeas);
            
            filter.setState(initState);
            
            if stateDecomp
                filter.setStateDecompDim(1);
            end
            
            filter.update(measModel, measurements);
            
            [stateMean, stateCov] = filter.getPointEstimate();
            
            test.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            test.verifyEqual(stateCov, stateCov');
            test.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
        
        function checkUpdateConfigKF(obj, config, test, filter)
            if ~islogical(config) || ~isequal(size(config), [1 4])
                error('Invalid configuration');
            end
            
            stateDecomp = config(1);
            multiMeas   = config(2);
            multiIter   = config(3);
            measGating  = config(4);
            
            [initState, measModel, measurements,  ...
             trueStateMean, trueStateCov, ...
             trueMeasMean, trueMeasCov, trueCrossCov] = obj.updateConfig(stateDecomp, multiMeas);
            
            tol = sqrt(eps);
            
            filter.setState(initState);
            
            if stateDecomp
                filter.setStateDecompDim(1);
            end
            
            if multiIter
                numIter = 3;
                filter.setMaxNumIterations(numIter);
            else
                numIter = 1;
            end
            
            if measGating
                % Compute threshold that should fail
                dimMeas  = size(trueMeasMean, 1);
                measDist = (trueMeasMean - measurements(:))' * trueMeasCov^(-1) * (trueMeasMean - measurements(:));
                
                normalizedDist = chi2cdf(measDist, dimMeas);
                
                filter.setMeasValidationThreshold(normalizedDist * 0.5);
                
                test.verifyWarning(@() filter.update(measModel, measurements), ...
                                   'Filter:IgnoringMeasurement');
                
                [stateMean, stateCov] = filter.getPointEstimate();
                
                [initStateMean, initStateCov] = initState.getMeanAndCovariance();
                
                % State estimate should not change
                test.verifyEqual(stateMean, initStateMean);
                test.verifyEqual(stateCov, stateCov');
                test.verifyEqual(stateCov, initStateCov);
            else
                filter.update(measModel, measurements);
                
                [stateMean, stateCov] = filter.getPointEstimate();
                
                test.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
                test.verifyEqual(stateCov, stateCov');
                test.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
            end
            
            [meas, ...
             measMean, ...
             measCov, ...
             stateMeasCrossCov, ...
             numIterations] = filter.getLastUpdateData();
            
            test.verifyEqual(meas, measurements(:), 'RelTol', tol);
            test.verifyEqual(measMean, trueMeasMean, 'RelTol', tol);
            test.verifyEqual(measCov, measCov');
            test.verifyEqual(measCov, trueMeasCov, 'RelTol', tol);
            test.verifyEqual(stateMeasCrossCov, trueCrossCov, 'RelTol', tol);
            
            test.verifyEqual(numIterations, numIter);
        end
    end
    
    methods (Abstract, Access = 'protected')
        [initState, measModel, measurements,  ...
         trueStateMean, trueStateCov, ...
         trueMeasMean, trueMeasCov, trueCrossCov] = updateConfig(obj, stateDecomp, multiMeas);
    end
end
