
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
            obj.verifyEqual(f.getNumParticles(), 1000);
            obj.verifyEqual(f.getMinAllowedNormalizedESS(), 0.5);
        end
        
        function testSetStateGaussian(obj)
            f = SIRPF();
            
            d = Gaussian(zeros(2, 1), diag([1.5, 2]));
            
            f.setState(d);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [2, 1000]);
            obj.verifySize(weights, [1, 1000]);
        end
        
        function testSetStateGaussianMixture(obj)
            f = SIRPF();
            
            d = GaussianMixture(ones(2, 2), cat(3, diag([1.5, 2]), 3 * eye(2)), [1 5]);
            
            f.setState(d);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [2, 1000]);
            obj.verifySize(weights, [1, 1000]);
        end
        
        function testSetStateUniform(obj)
            f = SIRPF();
            
            d = Uniform([-2 3 0], [5 10 1]);
            
            f.setState(d);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [3, 1000]);
            obj.verifySize(weights, [1, 1000]);
        end
        
        function testSetStateDiracMixture(obj)
            f = SIRPF();
            
            s = cat(2, zeros(3, 1), ones(3, 1), [-2 5 0.7]');
            w = [1 2 3];
            
            d = DiracMixture(s, w);
            
            f.setState(d);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifyEqual(samples, s);
            obj.verifyEqual(weights, w / sum(w));
        end
        
        function testSetStateJointDistribution(obj)
            f = SIRPF();
            
            dists = { Gaussian(), Uniform(), Gaussian(ones(2, 1), [2 0.5; 0.5 1.2]) };
            
            d = JointDistribution(dists);
            
            f.setState(d);
            
            dm = f.getState();
            
            [samples, weights] = dm.getComponents();
            
            obj.verifySize(samples, [4, 1000]);
            obj.verifySize(weights, [1, 1000]);
        end
        
        function testGetPointEstimate(obj)
            f = SIRPF();
            
            s = cat(2, zeros(3, 1), ones(3, 1), [-2 5 0.7]');
            w = [1 2 3];
            
            d = DiracMixture(s, w);
            
            f.setState(d);
            
            [pointEstimate, uncertainty] = f.getPointEstimate();
            
            [mean, cov] = d.getMeanAndCovariance();
            
            obj.verifyEqual(pointEstimate, mean);
            obj.verifyEqual(uncertainty, cov);
        end
    end
end
