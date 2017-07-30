
classdef TestPGF < matlab.unittest.TestCase & TestCopy
    % Provides unit tests for the PGF class.
    
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
            
            obj.verifyEqual(f.getName(), 'PGF');
            obj.verifyEqual(f.getMaxNumProgSteps(), 0);
        end
        
        
        function testPredictLinearSysModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictAddNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsAdditiveNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictSysModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsSystemModel.checkPrediction(obj,f, tol);
        end
        
        function testPredictMixedNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            testUtils = TestUtilsLinearMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getNumProgSteps(), 1);
        end
        
        function testUpdateLinearMeasModelStateDecomp(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            testUtils = TestUtilsLinearMeasurementModel();
            testUtils.checkUpdateStateDecomp(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getNumProgSteps(), 1);
        end
        
        
        function testUpdateAddNoiseMeasModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            testUtils = TestUtilsAdditiveNoiseMeasurementModel();
            testUtils.checkUpdate(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getNumProgSteps(), 1);
        end
        
        function testUpdateAddNoiseMeasModelStateDecomp(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            testUtils = TestUtilsAdditiveNoiseMeasurementModel();
            testUtils.checkUpdateStateDecomp(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getNumProgSteps(), 1);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = PGF();
        end
    end
end
