
classdef TestRPF < TestSIRPF
    % Provides unit tests for the RPF class.
    
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
        function testConstructorDefault(obj)
            f = obj.initFilter();
            
            obj.verifyEqual(f.getName(), 'RPF');
        end
        
        
        function testSetNumParticles(obj)
            f = obj.initFilter();
            
            f.setNumParticles(2000);
            
            obj.verifyEqual(f.getNumParticles(), 2000);
        end
        
        function testSetNumParticlesWithResampling(obj)
            f = obj.initFilter();
            
            d = Gaussian(zeros(2, 1), diag([1.5, 2]));
            f.setState(d);
            
            f.setNumParticles(2000);
            
            obj.verifyEqual(f.getNumParticles(), 2000);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [2, 2000]);
            obj.verifySize(weights, [1, 2000]);
            obj.verifyEqual(weights, repmat(1/2000, 1, 2000), 'AbsTol', 1e-14);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = RPF();
        end
    end
end
