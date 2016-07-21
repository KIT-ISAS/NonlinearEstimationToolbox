
classdef TestAnalyticKF < matlab.unittest.TestCase
    % Provides unit tests for the AnalyticKF class.
    
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
            
            obj.verifyEqual(f.getName(), 'Analytic KF');
        end
        
        
        function testPredictLinearSysModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsLinearSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsLinearMeasurementModel.checkUpdateKF(obj, f, tol, 1);
        end
        
        function testUpdateLinearMeasModelMultiIter(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setMaxNumIterations(3);
            
            TestUtilsLinearMeasurementModel.checkUpdateKF(obj, f, tol, 3);
        end
        
        function testUpdateLinearMeasModelStateDecomp(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setStateDecompDim(1);
            
            TestUtilsLinearMeasurementModel.checkUpdateKFStateDecomp(obj, f, tol, 1);
        end
        
        function testUpdateLinearMeasModelStateDecompMultiIter(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setStateDecompDim(1);
            f.setMaxNumIterations(3);
            
            TestUtilsLinearMeasurementModel.checkUpdateKFStateDecomp(obj, f, tol, 3);
        end
        
        function testUpdateLinearMeasModelMultiMeas(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsLinearMeasurementModel.checkUpdateKFMultiMeas(obj, f, tol, 1);
        end
        
        function testUpdateLinearMeasModelMultiMeasMultiIter(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setMaxNumIterations(3);
            
            TestUtilsLinearMeasurementModel.checkUpdateKFMultiMeas(obj, f, tol, 3);
        end
        
        function testUpdateLinearMeasModelMultiMeasStateDecomp(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setStateDecompDim(1);
            
            TestUtilsLinearMeasurementModel.checkUpdateKFMultiMeasStateDecomp(obj, f, tol, 1);
        end
        
        function testUpdateLinearMeasModelMultiMeasMultiIterStateDecomp(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            f.setStateDecompDim(1);
            f.setMaxNumIterations(3);
            
            TestUtilsLinearMeasurementModel.checkUpdateKFMultiMeasStateDecomp(obj, f, tol, 3);
        end
    end
    
    methods (Access = 'private')
        function f = initFilter(~)
            f = AnalyticKF();
        end
    end
end
