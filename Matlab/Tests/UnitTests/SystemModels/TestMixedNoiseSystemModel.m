
classdef TestMixedNoiseSystemModel < matlab.unittest.TestCase
    % Provides unit tests for the MixedNoiseSystemModel class.
    
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
        function testSimulate(obj)
            sysModel = MixedNoiseSysModel();
            sysModel.setAdditiveNoise(Uniform([0 0], [1 1]));
            sysModel.setNoise(Uniform([0 0], [1 1]));
            
            state = [0.3 -pi]';
            
            detSimState = sysModel.sysMatrix * state;
            
            simState = sysModel.simulate(state);
            
            obj.verifyEqual(size(simState), [2 1]);
            obj.verifyGreaterThanOrEqual(simState, detSimState);
            obj.verifyLessThanOrEqual(simState, detSimState + 2);
        end
        
        
        function testDerivative(obj)
            sysModel = MixedNoiseSysModel();
            
            nominalState = [-3 0.5]';
            nominalNoise = [1 -0.7]';
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = sysModel.derivative(nominalState, nominalNoise);
            
            obj.verifyEqual(stateJacobian, sysModel.sysMatrix, 'AbsTol', 1e-8);
            obj.verifyEqual(noiseJacobian, eye(2), 'AbsTol', 1e-8);
            
            obj.verifyEqual(stateHessians, zeros(2, 2, 2), 'AbsTol', 1e-8);
            obj.verifyEqual(noiseHessians, zeros(2, 2, 2), 'AbsTol', 1e-8);
        end
    end
end
