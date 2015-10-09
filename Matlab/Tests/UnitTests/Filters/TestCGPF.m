
classdef TestCGPF < matlab.unittest.TestCase
    % Provides unit tests for the CGPF class.
    
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
            
            obj.verifyEqual(f.getName(), 'CGPF');
            obj.verifyEqual(f.getNumParticles(), 1000);
        end
        
        
        function testPredictLinearSysModel(obj)
            f = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictAddNoiseSysModel(obj)
            f = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsAdditiveNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictSysModel(obj)
            f = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsSystemModel.checkPrediction(obj,f, tol);
        end
        
        function testPredictMixedNoiseSysModel(obj)
            f = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(1000000);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearMeasurementModel.checkUpdate(obj, f, tol);
        end
        
        function testUpdateLinearMeasModelMultiMeas(obj)
            f = obj.initFilter();
            tol = 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsLinearMeasurementModel.checkUpdateMultiMeas(obj, f, tol);
        end
        
        
        function testUpdateAddNoiseMeasModel(obj)
            f = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsAdditiveNoiseMeasurementModel.checkUpdate(obj, f, tol);
        end
        
        function testUpdateAddNoiseMeasModelMultiMeas(obj)
            f = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsAdditiveNoiseMeasurementModel.checkUpdateMultiMeas(obj, f, tol);
        end
        
        
        function testStepAddNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsStep.checkAdditiveNoiseSystemModel(obj, f, tol);
        end
        
        function testStepAddNoiseSysModelMultiMeas(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsStep.checkAdditiveNoiseSystemModelMultiMeas(obj, f, tol);
        end
        
        
        function testStepSysModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsStep.checkSystemModel(obj, f, tol);
        end
        
        function testStepSysModelMultiMeas(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsStep.checkSystemModelMultiMeas(obj, f, tol);
        end
        
        
        function testStepMixedNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsStep.checkMixedNoiseSystemModel(obj, f, tol);
        end
        
        function testStepMixedNoiseSysModelMultiMeas(obj)
            f   = obj.initFilter();
            tol = 5 * 1e-2;
            
            f.setNumParticles(5000000);
            
            TestUtilsStep.checkMixedNoiseSystemModelMultiMeas(obj, f, tol);
        end
    end
    
    methods (Access = 'private')
        function f = initFilter(~)
            f = CGPF();
        end
    end
end
