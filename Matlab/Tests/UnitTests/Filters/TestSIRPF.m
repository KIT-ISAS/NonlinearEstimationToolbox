
classdef TestSIRPF < matlab.unittest.TestCase
    % Provides unit tests for the SIRPF class.
    
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
            f = SIRPF();
            
            obj.verifyEqual(f.getName(), 'SIR-PF');
        end
        
        
        function testPredictLinearSysModel(obj)
            f   = SIRPF();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictAddNoiseSysModel(obj)
            f   = SIRPF();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsAdditiveNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictSysModel(obj)
            f   = SIRPF();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsSystemModel.checkPrediction(obj,f, tol);
        end
        
        function testPredictMixedNoisSysModel(obj)
            f   = SIRPF();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f   = SIRPF();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearMeasurementModel.checkUpdate(obj, f, tol);
        end
        
        function testUpdateLinearMeasModelMultiMeas(obj)
            f   = SIRPF();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearMeasurementModel.checkUpdateMultiMeas(obj, f, tol);
        end
        
        
        function testUpdateAddNoiseMeasModel(obj)
            f   = SIRPF();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsAdditiveNoiseMeasurementModel.checkUpdate(obj, f, tol);
        end
        
        function testUpdateAddNoiseMeasModelMultiMeas(obj)
            f   = SIRPF();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsAdditiveNoiseMeasurementModel.checkUpdateMultiMeas(obj, f, tol);
        end
    end
end
