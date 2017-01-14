
classdef TestCopy < matlab.unittest.TestCase
    % Provides unit tests for Filter copy methods.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2017  Jannik Steinbring <jannik.steinbring@kit.edu>
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
        function testCopy(obj)
            f = obj.initFilter();
            
            g = f.copy();
            
            obj.verifyEqual(g.getName(), f.getName());
        end
        
        function testCopyWithName(obj)
            f = obj.initFilter();
            
            g = f.copyWithName('Copy');
            
            obj.verifyEqual(g.getName(), 'Copy');
        end
    end
    
    methods (Abstract, Access = 'protected')
        f = initFilter(obj);
    end
end

