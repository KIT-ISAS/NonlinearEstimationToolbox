
classdef TestPGF < matlab.unittest.TestCase
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
            f = PGF();
            
            obj.verifyEqual(f.getName(), 'PGF');
            obj.verifyEqual(f.getMaxNumProgSteps(), 0);
        end
        
        
        function testPredictLinearSysModel(obj)
            f   = PGF();
            tol = sqrt(eps);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictAddNoiseSysModel(obj)
            f   = PGF();
            tol = sqrt(eps);
            
            TestUtilsAdditiveNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictSysModel(obj)
            f   = PGF();
            tol = sqrt(eps);
            
            TestUtilsSystemModel.checkPrediction(obj,f, tol);
        end
        
        function testPredictMixedNoisSysModel(obj)
            f   = PGF();
            tol = sqrt(eps);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f   = PGF();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            TestUtilsLinearMeasurementModel.checkUpdate(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getLastUpdateData(), 1);
        end
        
        function testUpdateLinearMeasModelMultiMeas(obj)
            f   = PGF();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            TestUtilsLinearMeasurementModel.checkUpdateMultiMeas(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getLastUpdateData(), 1);
        end
        
        
        function testUpdateAddNoiseMeasModel(obj)
            f   = PGF();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            TestUtilsAdditiveNoiseMeasurementModel.checkUpdate(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getLastUpdateData(), 1);
        end
        
        function testUpdateAddNoiseMeasModelMultiMeas(obj)
            f   = PGF();
            tol = 5 * 1e-2;
            
            f.setNumSamples(201);
            
            TestUtilsAdditiveNoiseMeasurementModel.checkUpdateMultiMeas(obj, f, tol);
            
            obj.verifyGreaterThanOrEqual(f.getLastUpdateData(), 1);
        end
    end
end
