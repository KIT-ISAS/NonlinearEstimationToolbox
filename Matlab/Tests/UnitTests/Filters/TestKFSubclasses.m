
classdef TestKFSubclasses < matlab.unittest.TestCase & TestCopy
    % Provides unit tests for subclasses of the KF class.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2016  Jannik Steinbring <jannik.steinbring@kit.edu>
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
            
            TestUtilsSystemModel.checkPrediction(obj, f, tol);
        end
        
        function testPredictMixedNoiseSysModel(obj)
            f   = obj.initFilter();
            tol = sqrt(eps);
            
            TestUtilsMixedNoiseSystemModel.checkPrediction(obj, f, tol);
        end
        
        
        function testUpdateLinearMeasModel(obj)
            testUtils = TestUtilsLinearMeasurementModel();
            
            createFilter = @() obj.initFilter;
            
            testUtils.checkUpdateKF(obj, createFilter);
        end
        
        function testUpdateAddNoiseMeasModel(obj)
            testUtils = TestUtilsAdditiveNoiseMeasurementModel();
            
            createFilter = @() obj.initFilter;
            
            testUtils.checkUpdateKF(obj, createFilter);
        end
        
        function testUpdateMeasModel(obj)
            testUtils = TestUtilsMeasurementModel();
            
            createFilter = @() obj.initFilter;
            
            testUtils.checkUpdateKF(obj, createFilter);
        end
        
        function testUpdateMixedNoiseMeasModel(obj)
            testUtils = TestUtilsMixedNoiseMeasurementModel();
            
            createFilter = @() obj.initFilter;
            
            testUtils.checkUpdateKF(obj, createFilter);
        end
    end
    
    methods (Abstract, Access = 'protected')
        f = initFilter(obj);
    end
end
