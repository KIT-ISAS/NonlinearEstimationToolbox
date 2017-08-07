
classdef TestASIRPF < TestSIRPF
    % Provides unit tests for the ASIRPF class.
    
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
        function testStepAdditiveNoiseSystemModel(obj)
            getStepData = @() TestUtilsStep.getAdditiveNoiseSystemModelData;
            
            obj.testStep(getStepData);
        end
        
        function testStepSystemModel(obj)
            getStepData = @() TestUtilsStep.getSystemModelData;
            
            obj.testStep(getStepData);
        end
        
        function testStepMixedNoiseSystemModel(obj)
            getStepData = @() TestUtilsStep.getMixedNoiseSystemModelData;
            
            obj.testStep(getStepData);
        end
    end
    
    methods (Access = 'protected')
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestFilter(f);
            
            % ASIRPF-related tests
            obj.verifyEqual(f.getName(), 'ASIRPF');
            obj.verifyEqual(f.getNumParticles(), 1000);
            obj.verifyEqual(f.getMinAllowedNormalizedESS(), 0.5);
        end
        
        function f = initFilter(~)
            f = ASIRPF();
        end
        
        function testStep(obj, getStepData)
            [initState, sysModel, ...
             measModel, measurement, ...
             trueStateMean, trueStateCov] = getStepData();
            
            f = obj.initFilter();
            f.setNumParticles(1e7);
            
            tol = 0.5;
            
            f.setState(initState);
            
            f.step(sysModel, measModel, measurement);
            
            [stateMean, stateCov] = f.getStateMeanAndCov();
            
            obj.verifyEqual(stateMean, trueStateMean, 'RelTol', tol);
            obj.verifyEqual(stateCov, stateCov');
            obj.verifyEqual(stateCov, trueStateCov, 'RelTol', tol);
        end
    end
end
