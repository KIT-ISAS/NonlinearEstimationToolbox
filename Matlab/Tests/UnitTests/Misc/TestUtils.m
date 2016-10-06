
classdef TestUtils < matlab.unittest.TestCase
    % Provides unit tests for the Utils class.
    
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
        function testGetMeanAndCov(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            
            trueMean = [-4 -4 -4]';
            trueCov  = diag(repmat(8 / 7, 1, 3));
            
            [mean, cov] = Utils.getMeanAndCov(samples);
            
            obj.verifyEqual(mean, trueMean);
            obj.verifyEqual(cov, trueCov);
        end
        
        function testGetMeanAndCovWithWeights(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1] / 8;
            
            trueMean = [-4 -4 -4]';
            trueCov  = eye(3);
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
             
            obj.verifyEqual(mean, trueMean);
            obj.verifyEqual(cov, trueCov);
        end
        
        function testGetMeanCovAndCrossCov(obj)
            stateMean    = zeros(2, 1);
            stateSamples = [zeros(2, 1) sqrt(2.5) * eye(2) -sqrt(2.5) * eye(2)];
            stateSamples = bsxfun(@plus, stateSamples, stateMean);
            measSamples  = stateSamples(1, :).^2 + stateSamples(2, :).^2;
            
            trueMeasMean          = 2;
            trueMeasCov           = 1;
            truestateMeasCrossCov = zeros(2, 1);
            
            [measMean, measCov, ...
             stateMeasCrossCov] = Utils.getMeanCovAndCrossCov(stateMean, stateSamples, ...
                                                              measSamples);
            
            obj.verifyEqual(measMean, trueMeasMean, 'AbsTol', 1e-12);
            obj.verifyEqual(measCov, trueMeasCov, 'AbsTol', 1e-12);
            obj.verifyEqual(stateMeasCrossCov, truestateMeasCrossCov);
        end
        
        function testGetMeanCovAndCrossCovWithWeights(obj)
            stateMean    = zeros(2, 1);
            stateSamples = [zeros(2, 1) sqrt(3) * eye(2) -sqrt(3) * eye(2)];
            stateSamples = bsxfun(@plus, stateSamples, stateMean);
            weights      = [1/3 1/6 1/6 1/6 1/6];
            measSamples  = stateSamples(1, :).^2 + stateSamples(2, :).^2;
            
            trueMeasMean          = 2;
            trueMeasCov           = 2;
            truestateMeasCrossCov = zeros(2, 1);
            
            [measMean, measCov, ...
             stateMeasCrossCov] = Utils.getMeanCovAndCrossCov(stateMean, stateSamples, ...
                                                              measSamples, weights);
            
            obj.verifyEqual(measMean, trueMeasMean, 'AbsTol', 1e-12);
            obj.verifyEqual(measCov, trueMeasCov, 'AbsTol', 1e-12);
            obj.verifyEqual(stateMeasCrossCov, truestateMeasCrossCov);
        end
        
        function testKalmanUpdate(obj)
            stateMean         = [1, -1]';
            stateCov          = [1.7 -0.5; -0.5 1.3];
            H                 = [1 1; 0 1];
            R                 = diag([2, 0.5]);
            measurement       = [-2 3]';
            measMean          = H * stateMean;
            measCov           = H * stateCov * H' + R;
            stateMeasCrossCov = stateCov * H';
            
            K        = stateMeasCrossCov / measCov;
            trueMean = stateMean + K * (measurement - measMean);
            trueCov  = (eye(2) - K * H) * stateCov;
            
            [mean, cov] = Utils.kalmanUpdate(stateMean, stateCov, measurement, ...
                                             measMean, measCov, stateMeasCrossCov);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdatePriorUncorrelated(obj)
            % 1D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7  0.0  0.0
                          0.0  1.3 -0.3
                          0.0 -0.3  2.0];
            
            updatedStateMeanA = 3.0;
            updatedStateCovA  = 0.9;
            
            trueMean = [ 3.0 -1.0 -2.0]';
            trueCov  = [ 0.9  0.0  0.0
                         0.0  1.3 -0.3
                         0.0 -0.3  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.0
                         -0.5  1.3  0.0
                          0.0  0.0  2.0];
            
            updatedStateMeanA = [ 3.0 2.0]';
            updatedStateCovA  = [ 0.9 -0.1
                                 -0.1  1.0];
            
            trueMean = [ 3.0  2.0 -2.0]';
            trueCov  = [ 0.9 -0.1  0.0
                        -0.1  1.0  0.0
                         0.0  0.0  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdateMeanChanged(obj)
            % 1D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.1
                         -0.5  1.3 -0.3
                          0.1 -0.3  2.0];
            
            updatedStateMeanA = -3.0;
            updatedStateCovA  =  1.7;
            
            stateMeanB = [-1.0 -2.0]' + ([-0.5  0.1]' / 1.7) * (-3.0 - 1.0);
            trueMean   = [-3.0 stateMeanB']';
            trueCov    = [ 1.7 -0.5  0.1
                          -0.5  1.3 -0.3
                           0.1 -0.3  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.1
                         -0.5  1.3 -0.3
                          0.1 -0.3  2.0];
            
            updatedStateMeanA = [ 3.0 2.0]';
            updatedStateCovA  = [ 1.7 -0.5
                                 -0.5  1.3];
            
            stateMeanB = -2.0 + ([0.1 -0.3] / [ 1.7 -0.5
                                               -0.5  1.3]) * ([ 3.0 2.0]' - [ 1.0 -1.0]');
            trueMean   = [ 3.0 2.0 stateMeanB']';
            trueCov    = [ 1.7 -0.5  0.1
                          -0.5  1.3 -0.3
                           0.1 -0.3  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdateCovChanged(obj)
            % 1D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.1
                         -0.5  1.3 -0.3
                          0.1 -0.3  2.0];
            
            updatedStateMeanA =  1.0;
            updatedStateCovA  =  0.9;
            
            K          = [-0.5  0.1]' / 1.7;
            covBA      = K * 0.9;
            covB       = [ 1.3 -0.3
                          -0.3  2.0] + K * (0.9 - 1.7) * K';
            trueMean   = [ 1.0 -1.0 -2.0]';
            trueCov    = [ 0.9  covBA'
                          covBA covB  ];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.1
                         -0.5  1.3 -0.3
                          0.1 -0.3  2.0];
            
            updatedStateMeanA = [ 1.0 -1.0]';
            updatedStateCovA  = [ 0.9 -0.1
                                 -0.1  1.0];
            
            K          = [ 0.1 -0.3] / [ 1.7 -0.5
                                        -0.5  1.3];
            covBA      = K * [ 0.9 -0.1
                              -0.1  1.0];
            covB       = 2.0 + K * ([ 0.9 -0.1
                                     -0.1  1.0] - [ 1.7 -0.5
                                                   -0.5  1.3]) * K';
            trueMean   = [ 1.0 -1.0 -2.0]';
            trueCov    = [[ 0.9 -0.1
                           -0.1  1.0] covBA'
                           covBA      covB  ];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdateEquivalentToKalmanUpdate(obj)
            % 1D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.1
                         -0.5  1.3 -0.3
                          0.1 -0.3  2.0];
            
            H                 = [ 1.0  0.0  0.0
                                 -0.5  0.0  0.0];
            R                 = diag([2, 0.5]);
            measurement       = [-2 3]';
            measMean          = H * stateMean;
            measCov           = H * stateCov * H' + R;
            stateMeasCrossCov = stateCov * H';
            
            K        = stateMeasCrossCov / measCov;
            trueMean = stateMean + K * (measurement - measMean);
            trueCov  = (eye(3) - K * H) * stateCov;
            
            updatedStateMeanA = trueMean(1);
            updatedStateCovA  = trueCov(1);
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean = [ 1.0 -1.0 -2.0]';
            stateCov  = [ 1.7 -0.5  0.1
                         -0.5  1.3 -0.3
                          0.1 -0.3  2.0];
            
            H                 = [ 1.0  1.0  0.0
                                 -0.5  0.0  0.0];
            R                 = diag([2, 0.5]);
            measurement       = [-2 3]';
            measMean          = H * stateMean;
            measCov           = H * stateCov * H' + R;
            stateMeasCrossCov = stateCov * H';
            
            K        = stateMeasCrossCov / measCov;
            trueMean = stateMean + K * (measurement - measMean);
            trueCov  = (eye(3) - K * H) * stateCov;
            
            updatedStateMeanA = trueMean(1:2);
            updatedStateCovA  = trueCov(1:2, 1:2);
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, ...
                                                            updatedStateMeanA, updatedStateCovA);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testKalmanUpdateInvalidMeasCov(obj)
            stateMean         = [];
            stateCov          = [];
            measurement       = [];
            measMean          = [];
            measCov           = ones(2, 2);
            stateMeasCrossCov = [];
            
            obj.verifyError(@() Utils.kalmanUpdate(stateMean, stateCov, measurement, ...
                                                   measMean, measCov, stateMeasCrossCov), ...
                            'Utils:InvalidMeasurementCovariance');
        end
        
        function testBlockDiag(obj)
            mat = [1 2; 3 4];
            n   = 3;
            
            blockMat = full(Utils.blockDiag(mat, n));
            
            trueMat = [1 2 0 0 0 0
                       3 4 0 0 0 0
                       0 0 1 2 0 0
                       0 0 3 4 0 0
                       0 0 0 0 1 2
                       0 0 0 0 3 4];
            
            obj.verifyEqual(blockMat, trueMat);
        end
        
        function testBaseBlockDiag(obj)
            matBase = [1 2; 3 4];
            matDiag = [2 -1; 3 -2];
            n       = 3;
            
            blockMat = Utils.baseBlockDiag(matBase, matDiag, n);
            
            trueMat = [3 1 1 2 1 2
                       6 2 3 4 3 4
                       1 2 3 1 1 2
                       3 4 6 2 3 4
                       1 2 1 2 3 1
                       3 4 3 4 6 2];
            
            obj.verifyEqual(blockMat, trueMat);
        end
        
        function testDrawGaussianRndSamples(obj)
            mean       = [1 -2]';
            cov        = [2 0.5; 0.5 1.2];
            numSamples = 10;
            
            covSqrt = chol(cov)';
            
            samples = Utils.drawGaussianRndSamples(mean, covSqrt, numSamples);
            
            obj.verifySize(samples, [2, numSamples]);
        end
        
        function testResampling(obj)
            samples    = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples    = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights    = [2 1 1 1 1 1 1] / 8;
            cumWeights = cumsum(weights);
            numSamples = 13;
            
            rndSamples = Utils.resampling(samples, cumWeights, numSamples);
            
            obj.verifySize(rndSamples, [3, numSamples]);
        end
        
        function testSystematicResampling(obj)
            samples    = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples    = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights    = [2 1 1 1 1 1 1] / 8;
            cumWeights = cumsum(weights);
            numSamples = 13;
            
            rndSamples = Utils.systematicResampling(samples, cumWeights, numSamples);
            
            obj.verifySize(rndSamples, [3, numSamples]);
        end
        
        function testRndOrthogonalMatrix(obj)
            rndMat = Utils.rndOrthogonalMatrix(4);
            
            identity = rndMat * rndMat';
            
            obj.verifyEqual(identity, eye(4), 'AbsTol', 1e-8);
        end
        
        function testGetStateSamples(obj)
            sampling = GaussianSamplingUKF();
            
            stateMean = [2 3]';
            stateCov  = [2 0.5; 0.5 1.2];
            stateCovSqrt = chol(stateCov)';
            
            [stateSamples, ...
             weights, ...
             numSamples] = Utils.getStateSamples(sampling, stateMean, stateCovSqrt);
            
            s = stateCovSqrt * sqrt(2.5) * [zeros(2, 1) -eye(2) eye(2)];
            s = bsxfun(@plus, s, stateMean);
            
            w = ones(1, 5) / 5;
            
            obj.verifyEqual(stateSamples, s, 'AbsTol', 1e-8);
            obj.verifyEqual(weights, w, 'AbsTol', 1e-8);
            obj.verifyEqual(numSamples, 5);
        end
        
        function testGetStateNoiseSamples(obj)
            sampling = GaussianSamplingUKF();
            
            stateMean    = 2;
            stateCovSqrt = sqrt(2);
            noiseMean    = 3;
            noiseCovSqrt = sqrt(1.2);
            
            [stateSamples, ...
             noiseSamples, ...
             weights, ...
             numSamples] = Utils.getStateNoiseSamples(sampling, stateMean, stateCovSqrt, ...
                                                      noiseMean, noiseCovSqrt);
            
            s = stateCovSqrt * sqrt(2.5) * [0 -1  0 1 0] + stateMean;
            n = noiseCovSqrt * sqrt(2.5) * [0  0 -1 0 1] + noiseMean;
            w = ones(1, 5) / 5;
            
            obj.verifyEqual(stateSamples, s, 'AbsTol', 1e-8);
            obj.verifyEqual(noiseSamples, n, 'AbsTol', 1e-8);
            obj.verifyEqual(weights, w, 'AbsTol', 1e-8);
            obj.verifyEqual(numSamples, 5);
        end
        
        function testDiffQuotientState(obj)
            nominalState = [-1.5 3.2]';
            
            [stateJacobian, stateHessians] = Utils.diffQuotientState(@TestUtils.stateFunc, nominalState);
            
            trueStateJacobian = TestUtils.jacobianStateFunc(nominalState);
            trueStateHessians = TestUtils.hessiansStateFunc(nominalState);
            
            obj.verifyEqual(stateJacobian, trueStateJacobian, 'RelTol', 1e-8);
            obj.verifyEqual(stateHessians, trueStateHessians, 'AbsTol', 1e-5);
        end
        
        function testDiffQuotientStateAndNoise(obj)
            nominalState = [-1.5 3.2]';
            nominalNoise = 2.7;
            
            [stateJacobian, ...
             noiseJacobian ] = Utils.diffQuotientStateAndNoise(@TestUtils.stateAndNoiseFunc, ...
                                                               nominalState, nominalNoise);
            
            [trueStateJacobian, ...
             trueNoiseJacobian] = TestUtils.diffStateAndNoiseFunc(nominalState, nominalNoise);
            
            obj.verifyEqual(stateJacobian, trueStateJacobian, 'AbsTol', 1e-4);
            obj.verifyEqual(noiseJacobian, trueNoiseJacobian, 'AbsTol', 1e-4);
        end
    end
    
    methods (Static, Access = 'private')
        function values = stateFunc(states)
            values = [states(1, :).^2 .* states(2, :).^3
                      states(1, :) + sin(states(2, :))
                      exp(states(1, :)) .* sqrt(states(2, :))];
        end
        
        function stateJacobian = jacobianStateFunc(state)
            stateJacobian = [2 * state(1) * state(2)^3        	3 * state(1)^2 * state(2)^2
                             1                               	cos(state(2))
                             exp(state(1)) * sqrt(state(2))   	0.5 * exp(state(1)) * state(2)^(-0.5)];
        end
        
        function stateHessians = hessiansStateFunc(state)
            stateHessians(:, :, 1) = [2 * state(2)^3                6 * state(1) * state(2)^2
                                      6 * state(1) * state(2)^2   	6 * state(1)^2 * state(2)];
            stateHessians(:, :, 2) = [0        0
                                      0        -sin(state(2))];
            stateHessians(:, :, 3) = [exp(state(1)) * sqrt(state(2))        	0.5 * exp(state(1)) * state(2)^(-0.5)
                                      0.5 * exp(state(1)) * state(2).^(-0.5)	-1/4 * exp(state(1)) * state(2)^(-1.5)];
        end
        
        function values = stateAndNoiseFunc(states, noise)
            values = [states(1, :).^2 .* states(2, :).^3 .* noise.^2
                      states(1, :) + sin(states(2, :)) .* sqrt(noise)];
        end
        
        function [stateJacobian, noiseJacobian] = diffStateAndNoiseFunc(states, noise)
            stateJacobian = [2 * states(1, :) .* states(2, :).^3 .* noise.^2 states(1, :).^2 .* 3 * states(2, :).^2 .* noise.^2
                             1                                               cos(states(2, :)) .* sqrt(noise)                  ];
            
            noiseJacobian = [states(1, :).^2 .* states(2, :).^3 .* 2 * noise
                             sin(states(2, :)) .* 0.5 * noise.^(-0.5)       ];
        end
    end
end
