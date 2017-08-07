
classdef TestPGF < TestGaussianFilter
    % Provides unit tests for the PGF class.
    
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
        function testSetMaxNumProgSteps(obj)
            f = obj.initFilter();
            
            f.setMaxNumProgSteps(5);
            
            obj.verifyEqual(f.getMaxNumProgSteps(), 5);
        end
        
        
        function testSetNumSamples(obj)
            f = obj.initFilter();
            
            f.setNumSamples(201);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEqual(numSamplesAbs, 201);
            obj.verifyEmpty(numSamplesFactor);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEqual(numSamplesAbs, 201);
            obj.verifyEmpty(numSamplesFactor);
        end
        
        function testSetNumSamplesBoth(obj)
            f = obj.initFilter();
            
            f.setNumSamples(201, 501);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEqual(numSamplesAbs, 201);
            obj.verifyEmpty(numSamplesFactor);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEqual(numSamplesAbs, 501);
            obj.verifyEmpty(numSamplesFactor);
        end
        
        function testSetNumSamplesMeasEmpty(obj)
            f = obj.initFilter();
            
            f.setNumSamples(201, []);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEqual(numSamplesAbs, 201);
            obj.verifyEmpty(numSamplesFactor);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
        end
        
        function testSetNumSamplesPredEmpty(obj)
            f = obj.initFilter();
            
            f.setNumSamples([], 501);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEqual(numSamplesAbs, 501);
            obj.verifyEmpty(numSamplesFactor);
        end
        
        function testSetNumSamplesBothEmpty(obj)
            f = obj.initFilter();
            
            f.setNumSamples([], []);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
        end
        
        
        function testSetNumSamplesByFactors(obj)
            f = obj.initFilter();
            
            f.setNumSamplesByFactors(5);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 5);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 5);
        end
        
        function testSetNumSamplesByFactorsBoth(obj)
            f = obj.initFilter();
            
            f.setNumSamplesByFactors(5, 20);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 5);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 20);
        end
        
        function testSetNumSamplesByFactorsMeasEmpty(obj)
            f = obj.initFilter();
            
            f.setNumSamplesByFactors(5, []);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 5);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
        end
        
        function testSetNumSamplesByFactorsPredEmpty(obj)
            f = obj.initFilter();
            
            f.setNumSamplesByFactors([], 20);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 20);
        end
        
        function testSetNumSamplesByFactorsBothEmpty(obj)
            f = obj.initFilter();
            
            f.setNumSamplesByFactors([], []);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
        end
        
        
        function testUpdateLinearMeasurementModel(obj)
            configs    = logical(dec2bin(0:7) - '0');
            numConfigs = 8;
            
            for i = 1:numConfigs
                obj.testUpdateLinearMeasurementModelConfiguration(configs(i, :));
            end
        end
        
        function testUpdateAdditiveNoiseMeasurementModel(obj)
            % Without state decompoosition
            obj.testUpdateAdditiveNoiseMeasurementModelConfiguration(false);
            
            % With state decomposition
            obj.testUpdateAdditiveNoiseMeasurementModelConfiguration(true);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = PGF();
        end
        
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestGaussianFilter(f);
            
            % PGF-related tests
            obj.verifyEqual(f.getName(), 'PGF');
            obj.verifyEqual(f.getMaxNumProgSteps(), 0);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigPrediction();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
            
            [numSamplesAbs, ...
             numSamplesFactor] = f.getNumSamplesConfigUpdate();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
        end
        
        function [f, tol] = setupNonlinarPrediction(obj)
            f = obj.initFilter();
            
            tol = sqrt(eps);
        end
        
        function testUpdateLinearMeasurementModelConfiguration(obj, config)
            hasMeasMatrix  = config(1);
            stateDecomp    = config(2);
            postProcessing = config(3);
            
            [initState, measModel, ...
             measurement, stateDecompDim, ...
             trueStateMean, trueStateCov] = TestUtilsLinearMeasurementModel.getMeasModelData(hasMeasMatrix, stateDecomp);
            
            f = obj.initFilter();
            f.setNumSamples(501);
            
            tol = 1e-2;
            
            f.setState(initState);
            
            if stateDecomp
                f.setStateDecompDim(stateDecompDim);
            end
            
            if postProcessing
                f.setUpdatePostProcessing(@TestGaussianFilter.postProcessingScale);
            end
            
            f.update(measModel, measurement);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            if postProcessing
                [trueStateMean, ...
                 trueStateCov] = TestGaussianFilter.postProcessingScale(trueStateMean, trueStateCov);
            end
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
            
            obj.verifyGreaterThanOrEqual(f.getNumProgSteps(), 1);
        end
        
        function testUpdateAdditiveNoiseMeasurementModelConfiguration(obj, stateDecomp)
            [initState, measModel, ...
             measurement, stateDecompDim, ...
             trueStateMean, trueStateCov] = TestUtilsAdditiveNoiseMeasurementModel.getMeasModelData(stateDecomp);
            
            f = obj.initFilter();
            f.setNumSamples(501);
            
            tol = 1e-2;
            
            f.setState(initState);
            
            if stateDecomp
                f.setStateDecompDim(stateDecompDim);
            end
            
            f.update(measModel, measurement);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
            
            obj.verifyGreaterThanOrEqual(f.getNumProgSteps(), 1);
        end
    end
end
