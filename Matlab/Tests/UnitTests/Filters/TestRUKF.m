
classdef TestRUKF < TestKFSubclasses
    % Provides unit tests for the RUKF class.
    
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
            
            obj.verifyEqual(f.getName(), 'RUKF');
            
            [numIterationsPrediction, numIterationsUpdate] = f.getNumIterations();
            
            obj.verifyEqual(numIterationsPrediction, 5);
            obj.verifyEqual(numIterationsUpdate, 5);
        end
        
        function testSetNumIterationsDefault(obj)
            f = obj.initFilter();
            
            f.setNumIterations(10);
            
            [numIterationsPrediction, numIterationsUpdate] = f.getNumIterations();
            
            obj.verifyEqual(numIterationsPrediction, 10);
            obj.verifyEqual(numIterationsUpdate, 10);
        end
        
        function testSetNumIterations(obj)
            f = obj.initFilter();
            
            f.setNumIterations(10, 6);
            
            [numIterationsPrediction, numIterationsUpdate] = f.getNumIterations();
            
            obj.verifyEqual(numIterationsPrediction, 10);
            obj.verifyEqual(numIterationsUpdate, 6);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = RUKF();
        end
    end
end
