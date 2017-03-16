
classdef TestMixedNoiseMeasurementModel < matlab.unittest.TestCase
    % Provides unit tests for the MixedNoiseMeasurementModel class.
    
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
        function testSimulateDefaultNumMeasurements(obj)
            measModel = MixedNoiseMeasModel();
            measModel.setAdditiveNoise(Uniform([0 0 0], [1 1 1]));
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            state = [0.3 -pi]';
            
            detMeas = measModel.measMatrix * state;
            
            measurements = measModel.simulate(state);
            
            obj.verifyEqual(size(measurements), [3 1]);
            obj.verifyGreaterThanOrEqual(measurements, detMeas);
            obj.verifyLessThanOrEqual(measurements, detMeas + 2);
        end
        
        function testSimulate(obj)
            measModel = MixedNoiseMeasModel();
            measModel.setAdditiveNoise(Uniform([0 0 0], [1 1 1]));
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            state = [0.3 -pi]';
            
            detMeas = measModel.measMatrix * state;
            
            n = 3;
            
            measurements = measModel.simulate(state, n);
            
            obj.verifyEqual(size(measurements), [3 n]);
            obj.verifyGreaterThanOrEqual(measurements, repmat(detMeas, 1, n));
            obj.verifyLessThanOrEqual(measurements, repmat(detMeas, 1, n) + 2);
        end
        
        
        function testDerivative(obj)
            measModel = MixedNoiseMeasModel();
            
            nominalState = [-3 0.5]';
            nominalNoise = [1 -0.7 2.3]';
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = measModel.derivative(nominalState, nominalNoise);
            
            obj.verifyEqual(stateJacobian, measModel.measMatrix, 'AbsTol', 1e-8);
            obj.verifyEqual(noiseJacobian, eye(3), 'AbsTol', 1e-8);
            
            obj.verifyEqual(stateHessians, zeros(2, 2, 3), 'AbsTol', 1e-8);
            obj.verifyEqual(noiseHessians, zeros(3, 3, 3), 'AbsTol', 1e-8);
        end
    end
end
