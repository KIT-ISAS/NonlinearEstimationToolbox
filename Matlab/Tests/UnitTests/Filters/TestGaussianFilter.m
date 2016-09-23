
classdef TestGaussianFilter < matlab.unittest.TestCase
    % Provides unit tests for the GaussianFilter class.
    
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
        function testConstructor(obj)
            f = GaussianFilterStub();
            
            obj.verifyEqual(f.getName(), 'GaussianFilterStub');
            obj.verifyFalse(f.getUseAnalyticSystemModel());
            obj.verifyEqual(f.getStateDecompDim(), 0);
        end
        
        
        function testSetStateGaussian(obj)
            f = GaussianFilterStub();
            
            mean = zeros(2, 1);
            cov  = diag([1.5, 2]);
            
            d = Gaussian(mean, cov);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateGaussianMixture(obj)
            f = GaussianFilterStub();
            
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.31;
            w2 = 0.967;
            
            means   = [m1 m2];
            covs    = cat(3, c1, c2);
            weights = [w1 w2];
            
            d = GaussianMixture(means, covs, weights);
            
            weights  = weights / (w1 + w2);
            w1       = weights(1);
            w2       = weights(2);
            mean     = w1 * m1 + w2 * m2;
            cov      = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - mean) * (m1 - mean)' + ...
                       w2 * (m2 - mean) * (m2 - mean)';
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateUniform(obj)
            f = GaussianFilterStub();
            
            a = [-1, 3]';
            b = [-0.5, 5]';
            d = Uniform(a, b);
            
            mean = [-0.75, 4]';
            cov  = diag([0.25 4] / 12);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 2);
        end
        
        function testSetStateDiracMixture(obj)
            f = GaussianFilterStub();
            
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1];
            
            d = DiracMixture(samples, weights);
            
            mean = [-4 -4 -4]';
            cov  = eye(3);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 3);
        end
        
        function testSetStateJointDistribution(obj)
            f = GaussianFilterStub();
            
            dists = { Gaussian(), Uniform(), Gaussian(ones(2, 1), [2 0.5; 0.5 1.2]) };
            
            d = JointDistribution(dists);
            
            mean = [0, 0.5, 1, 1]';
            cov  = blkdiag(1, 1/12, [2 0.5; 0.5 1.2]);
            
            f.setState(d);
            
            obj.checkState(f, mean, cov, 4);
        end
        
        
        function testGetPointEstimate(obj)
            f = GaussianFilterStub();
            
            a = [-1, 3]';
            b = [-0.5, 5]';
            d = Uniform(a, b);
            
            mean = [-0.75, 4]';
            cov  = diag([0.25 4] / 12);
            
            f.setState(d);
            
            [pointEstimate, uncertainty] = f.getPointEstimate();
            
            obj.verifyEqual(pointEstimate, mean);
            obj.verifyEqual(uncertainty, cov);
        end
    end
    
    methods (Access = 'private')
        function checkState(obj, f, mean, cov, dim)
            state = f.getState();
            
            obj.verifyClass(state, 'Gaussian');
            
            [stateMean, stateCov] = state.getMeanAndCovariance();
            
            obj.verifyEqual(stateMean, mean, 'Abstol', 1e-8);
            obj.verifyEqual(stateCov, cov, 'Abstol', 1e-8);
            
            obj.verifyEqual(f.getStateDim(), dim);
        end
    end
end
