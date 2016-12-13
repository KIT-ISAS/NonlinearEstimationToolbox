
classdef TestJointDistribution < matlab.unittest.TestCase
    % Provides unit tests for the JointDistribution class.
    
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
        function testConstructorSingleDist(obj)
            dists = Gaussian([2 -1]', [3 1.2]);
            
            j = JointDistribution(dists);
            
            dim      = 2;
            dists    = { dists };
            numDists = 1;
            dimDists = 2;
            mean     = [2 -1]';
            cov      = diag([3 1.2]);            
            covSqrt  = chol(cov)';
            
            obj.verifyJoint(j, dim, dists, numDists, dimDists, mean, cov, covSqrt);
        end
        
        function testConstructorSingleDistCell(obj)
            dists = { Gaussian([2 -1]', [3 1.2]) };
            
            j = JointDistribution(dists);
            
            dim      = 2;
            numDists = 1;
            dimDists = 2;
            mean     = [2 -1]';
            cov      = diag([3 1.2]);            
            covSqrt  = chol(cov)';
            
            obj.verifyJoint(j, dim, dists, numDists, dimDists, mean, cov, covSqrt);
        end
        
        function testConstructorMultipleDists(obj)
            dists = { Gaussian(), Uniform(), Gaussian(ones(2, 1), [2 0.5; 0.5 1.2]) };
            
            j = JointDistribution(dists);
            
            dim      = 4;
            numDists = 3;
            dimDists = [1 1 2];
            mean     = [0 0.5 1 1]';
            cov      = [1 0    0   0
                        0 1/12 0   0
                        0 0    2   0.5
                        0 0    0.5 1.2];
            covSqrt  = chol(cov)';
            
            obj.verifyJoint(j, dim, dists, numDists, dimDists, mean, cov, covSqrt);
        end
        
        function testConstructorInvalidDist(obj)
            obj.verifyError(@() JointDistribution('A'), ...
                            'JointDistribution:InvalidDistributions');
            
            obj.verifyError(@() JointDistribution(ones(2, 3)), ...
                            'JointDistribution:InvalidDistributions');
            
            obj.verifyError(@() JointDistribution({ Gaussian(), ones(2, 3) }), ...
                            'JointDistribution:InvalidDistributions');
        end
        
        function testDrawRndSamples(obj)
            dists = { Gaussian(), Uniform(-1.5, 2), Gaussian(ones(2, 1), 2 * eye(2)) };
            
            j = JointDistribution(dists);
            
            samples = j.drawRndSamples(9);
            
            obj.verifySize(samples, [4, 9]);
            
            obj.verifyGreaterThanOrEqual(samples(2, :), -1.5);
            obj.verifyLessThanOrEqual(samples(2, :), 2);
        end
        
        function testDrawRndSamplesInvalidNumSamples(obj)
            dists = { Gaussian(), Uniform(-1.5, 2), Gaussian(ones(2, 1), 2 * eye(2)) };
            
            j = JointDistribution(dists);
            
            obj.verifyError(@() j.drawRndSamples(-3.4), ...
                            'JointDistribution:InvalidNumberOfSamples');
            
            obj.verifyError(@() j.drawRndSamples(eye(2)), ...
                            'JointDistribution:InvalidNumberOfSamples');
            
            obj.verifyError(@() j.drawRndSamples('alkdjf'), ...
                            'JointDistribution:InvalidNumberOfSamples');
        end
        
        function testLogPdf(obj)
            mean  = [1 -2]';
            cov   = [2 0.5; 0.5 1.2];
            dists = { Uniform(-1.5, 2.3), Gaussian(mean, cov) };
            
            j = JointDistribution(dists);
            
            values = [-3.0  2.3 5.0   1.2
                       0.1 -0.5 13.4 -4.3
                       0.9 -10  5.0   0.01];
            
            logValues = j.logPdf(values);
            
            obj.verifySize(logValues, [1, 4]);
            
            % True uniform log PDF values
            c = 1 / 3.8;
            trueUniformLogValues = log([0 c 0 c]);
            
            % True Gaussian log PDF values
            det    = cov(1, 1) * cov(2, 2) - cov(2, 1) * cov(1, 2);
            invCov = [cov(2, 2) -cov(1, 2); -cov(2, 1) cov(1, 1)] / det;
            c      = (1 / (2 * pi * sqrt(abs(det))));
            v      = bsxfun(@minus, values(2:3, :), mean);
            
            trueGaussianLogValues = log(c * exp(-0.5 * diag(v' * invCov * v)'));
            
            % Verify log PDF values
            trueLogValues = trueUniformLogValues + trueGaussianLogValues;
            
            obj.verifyEqual(logValues, trueLogValues, 'AbsTol', 1e-10);
        end
        
        function testLogPdfInvalidValues(obj)
            dists = { Uniform(-1.5, 2), Gaussian(ones(2, 1), 2 * eye(2)) };
            
            j = JointDistribution(dists);
            
            obj.verifyError(@() j.logPdf(ones(2, 10)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() j.logPdf(ones(3, 10, 5)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() j.logPdf('42kd'), ...
                            'Distribution:InvalidValues');  
        end
    end
    
    methods (Access = 'private')
        function verifyJoint(obj, j, dim, dists, numDists, dimDists, mean, cov, covSqrt)
            d = j.getDimension();
            obj.verifyEqual(d, dim);
            
            [ds, n, d] = j.getDistributions();
            obj.verifyEqual(ds, dists);
            obj.verifyEqual(n, numDists);
            obj.verifyEqual(d, dimDists);
            
            [m, c, cSqrt] = j.getMeanAndCovariance();
            obj.verifyEqual(m, mean);
            obj.verifyEqual(c, c');
            obj.verifyEqual(c, cov);
            obj.verifyEqual(cSqrt, covSqrt);
        end
    end
end
