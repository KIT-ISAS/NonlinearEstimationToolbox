
classdef TestGHKF < TestKFSubclasses
    % Provides unit tests for the GHKF class.
    
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
            
            obj.verifyEqual(f.getName(), 'GHKF');
            
            [numPointsPrediction, numPointsUpdate] = f.getNumQuadraturePoints();
            
            obj.verifyEqual(numPointsPrediction, 2);
            obj.verifyEqual(numPointsUpdate, 2);
        end
        
        function testSetNumQuadraturePointsDefault(obj)
            f = obj.initFilter();
            
            f.setNumQuadraturePoints(3);
            
            [numPointsPrediction, numPointsUpdate] = f.getNumQuadraturePoints();
            
            obj.verifyEqual(numPointsPrediction, 3);
            obj.verifyEqual(numPointsUpdate, 3);
        end
        
        function testSetNumQuadraturePoints(obj)
            f = obj.initFilter();
            
            f.setNumQuadraturePoints(4, 3);
            
            [numPointsPrediction, numPointsUpdate] = f.getNumQuadraturePoints();
            
            obj.verifyEqual(numPointsPrediction, 4);
            obj.verifyEqual(numPointsUpdate, 3);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = GHKF();
        end
    end
end
