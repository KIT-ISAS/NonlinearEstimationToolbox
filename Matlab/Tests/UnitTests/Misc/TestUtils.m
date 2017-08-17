
classdef TestUtils < matlab.unittest.TestCase
    % Provides unit tests for the Utils class.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
    %                             Florian Rosenthal <florian.rosenthal@kit.edu>
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
        function testGetMeanAndCovNoWeights(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            
            trueMean = [-4 -4 -4]';
            trueCov  = diag(repmat(8 / 7, 1, 3));
            
            [mean, cov] = Utils.getMeanAndCov(samples);
            
            obj.verifyEqual(mean, trueMean);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov);
        end
        
        function testGetMeanAndCovWithSingleWeight(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = 1 / 7;
            
            trueMean = [-4 -4 -4]';
            trueCov  = diag(repmat(8 / 7, 1, 3));
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
            
            obj.verifyEqual(mean, trueMean);
            obj.verifyEqual(cov, cov');
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
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testGetMeanAndCovWithNegativeWeights(obj)
            samples = [zeros(3, 1) sqrt(2) * eye(3) -sqrt(2) * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [-0.5 0.25 * ones(1, 6)];
            
            trueMean = [-4 -4 -4]';
            trueCov  = eye(3);
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
            
            obj.verifyEqual(mean, trueMean);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
        end
        
        
        function testGetGMMeanAndCovSingleComponentNoWeights(obj)
            means   = [1 -2]';
            covs    = [2 0.5; 0.5 1.2];
            
            trueMean = [1 -2]';
            trueCov  = [2 0.5; 0.5 1.2];
            
            [mean, cov] = Utils.getGMMeanAndCov(means, covs);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testGetGMMeanAndCovSingleComponentWithWeights(obj)
            means   = [1 -2]';
            covs    = [2 0.5; 0.5 1.2];
            weights = 1;
            
            trueMean = [1 -2]';
            trueCov  = [2 0.5; 0.5 1.2];
            
            [mean, cov] = Utils.getGMMeanAndCov(means, covs, weights);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testGetGMMeanAndCovNoWeights(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.5;
            w2 = 0.5;
            
            means = [m1 m2];
            covs  = cat(3, c1, c2);
            
            trueMean = w1 * m1 + w2 * m2;
            trueCov  = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - trueMean) * (m1 - trueMean)' + ...
                       w2 * (m2 - trueMean) * (m2 - trueMean)';
            
            [mean, cov] = Utils.getGMMeanAndCov(means, covs);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testGetGMMeanAndCovWithWeights(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.25;
            w2 = 0.75;
            
            means   = [m1 m2];
            covs    = cat(3, c1, c2);
            weights = [w1 w2];
            
            trueMean = w1 * m1 + w2 * m2;
            trueCov  = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - trueMean) * (m1 - trueMean)' + ...
                       w2 * (m2 - trueMean) * (m2 - trueMean)';
            
            [mean, cov] = Utils.getGMMeanAndCov(means, covs, weights);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
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
            
            innovation          = measurement - measMean;
            K                   = stateMeasCrossCov / measCov;
            trueMean            = stateMean + K * innovation;
            trueCov             = (eye(2) - K * H) * stateCov;
            trueSqMeasMahalDist = innovation' * measCov^(-1) * innovation;
            
            [mean, cov, ...
             sqMeasMahalDist] = Utils.kalmanUpdate(stateMean, stateCov, measurement, ...
                                                   measMean, measCov, stateMeasCrossCov);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(cov, cov');
            obj.verifyEqual(cov, trueCov, 'AbsTol', 1e-12);
            obj.verifyEqual(sqMeasMahalDist, trueSqMeasMahalDist, 'AbsTol', 1e-12);
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
        
        
        function testDecomposedStateUpdatePriorUncorrelated(obj)
            % 1D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7  0.0  0.0
                             0.0  1.3 -0.3
                             0.0 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            updatedStateMeanA    = 3.0;
            updatedStateCovA     = 0.9;
            updatedStateCovASqrt = sqrt(updatedStateCovA);
            
            trueMean = [ 3.0 -1.0 -2.0]';
            trueCov  = [ 0.9  0.0  0.0
                         0.0  1.3 -0.3
                         0.0 -0.3  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.0
                            -0.5  1.3  0.0
                             0.0  0.0  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            updatedStateMeanA    = [ 3.0 2.0]';
            updatedStateCovA     = [ 0.9 -0.1
                                    -0.1  1.0];
            updatedStateCovASqrt = chol(updatedStateCovA, 'Lower');
            
            trueMean = [ 3.0  2.0 -2.0]';
            trueCov  = [ 0.9 -0.1  0.0
                        -0.1  1.0  0.0
                         0.0  0.0  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdateMeanChanged(obj)
            % 1D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.1
                            -0.5  1.3 -0.3
                             0.1 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            updatedStateMeanA    = -3.0;
            updatedStateCovA     =  1.7;
            updatedStateCovASqrt = sqrt(updatedStateCovA);
            
            stateMeanB = [-1.0 -2.0]' + ([-0.5  0.1]' / 1.7) * (-3.0 - 1.0);
            trueMean   = [-3.0 stateMeanB']';
            trueCov    = [ 1.7 -0.5  0.1
                          -0.5  1.3 -0.3
                           0.1 -0.3  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.1
                            -0.5  1.3 -0.3
                             0.1 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            updatedStateMeanA    = [ 3.0 2.0]';
            updatedStateCovA     = [ 1.7 -0.5
                                    -0.5  1.3];
            updatedStateCovASqrt = chol(updatedStateCovA, 'Lower');
            
            stateMeanB = -2.0 + ([0.1 -0.3] / [ 1.7 -0.5
                                               -0.5  1.3]) * ([ 3.0 2.0]' - [ 1.0 -1.0]');
            trueMean   = [ 3.0 2.0 stateMeanB']';
            trueCov    = [ 1.7 -0.5  0.1
                          -0.5  1.3 -0.3
                           0.1 -0.3  2.0];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdateCovChanged(obj)
            % 1D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.1
                            -0.5  1.3 -0.3
                             0.1 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            updatedStateMeanA    =  1.0;
            updatedStateCovA     =  0.9;
            updatedStateCovASqrt = sqrt(updatedStateCovA);
            
            K          = [-0.5  0.1]' / 1.7;
            covBA      = K * 0.9;
            covB       = [ 1.3 -0.3
                          -0.3  2.0] + K * (0.9 - 1.7) * K';
            trueMean   = [ 1.0 -1.0 -2.0]';
            trueCov    = [ 0.9  covBA'
                          covBA covB  ];
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.1
                            -0.5  1.3 -0.3
                             0.1 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            updatedStateMeanA    = [ 1.0 -1.0]';
            updatedStateCovA     = [ 0.9 -0.1
                                    -0.1  1.0];
            updatedStateCovASqrt = chol(updatedStateCovA, 'Lower');
            
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
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
        end
        
        function testDecomposedStateUpdateEquivalentToKalmanUpdate(obj)
            % 1D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.1
                            -0.5  1.3 -0.3
                             0.1 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            H                 = [ 1.0  0.0  0.0
                                 -0.5  0.0  0.0];
            R                 = diag([2, 0.5]);
            measurement       = [-2 3]';
            measMean          = H * stateMean;
            measCov           = H * stateCov * H' + R;
            stateMeasCrossCov = stateCov * H';
            
            sqrtMeasCov = chol(measCov);
            A = stateMeasCrossCov / sqrtMeasCov;
            K = A / sqrtMeasCov';
            trueMean = stateMean + K * (measurement - measMean);
            trueCov  = stateCov - A * A';
            
            updatedStateMeanA    = trueMean(1);
            updatedStateCovA     = trueCov(1);
            updatedStateCovASqrt = sqrt(updatedStateCovA);
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
            
            % 2D subspace A
            stateMean    = [ 1.0 -1.0 -2.0]';
            stateCov     = [ 1.7 -0.5  0.1
                            -0.5  1.3 -0.3
                             0.1 -0.3  2.0];
            stateCovSqrt = chol(stateCov, 'Lower');
            
            H                 = [ 1.0  1.0  0.0
                                 -0.5  0.0  0.0];
            R                 = diag([2, 0.5]);
            measurement       = [-2 3]';
            measMean          = H * stateMean;
            measCov           = H * stateCov * H' + R;
            stateMeasCrossCov = stateCov * H';
            
            sqrtMeasCov = chol(measCov);
            A = stateMeasCrossCov / sqrtMeasCov;
            K = A / sqrtMeasCov';
            trueMean = stateMean + K * (measurement - measMean);
            trueCov  = stateCov - A * A';
            
            updatedStateMeanA    = trueMean(1:2);
            updatedStateCovA     = trueCov(1:2, 1:2);
            updatedStateCovASqrt = chol(updatedStateCovA, 'Lower');
            
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(stateMean, stateCov, stateCovSqrt, ...
                                                            updatedStateMeanA, updatedStateCovA, updatedStateCovASqrt);
            
            obj.verifyEqual(updatedStateMean, trueMean, 'AbsTol', 1e-12);
            obj.verifyEqual(updatedStateCov, updatedStateCov');
            obj.verifyEqual(updatedStateCov, trueCov, 'AbsTol', 1e-12);
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
        
        
        function testGetGaussianKLDScalar(obj)
            meanA    =   1;
            meanB    =  10;
            covA     = 100;
            covSqrtA = sqrt(covA);
            covB     = 200;
            covSqrtB = sqrt(covB);
            
            trueValue = 0.2990735902799726547;
            
            value = Utils.getGaussianKLD(meanA, meanB, covA, covSqrtA, covSqrtB);
            
            obj.verifyEqual(value, trueValue, 'AbsTol', 1e-8);
        end
        
        function testGetGaussianKLDMultivariate(obj)
            meanA    = [1 1]';
            meanB    = [2 2]';
            covA     = gallery('moler', 2);
            covSqrtA = chol(covA, 'Lower');
            covB     = 0.5 * eye(2);
            covSqrtB = chol(covB, 'Lower');
            
            trueValue = 3.30685281944005469058;
            
            value = Utils.getGaussianKLD(meanA, meanB, covA, covSqrtA, covSqrtB);
            
            obj.verifyEqual(value, trueValue, 'AbsTol', 1e-8);
        end
        
        function testGetGaussianKLDIdentical(obj)
            meanA    = [1 1]';
            covA     = gallery('moler', 2);
            covSqrtA = chol(covA, 'Lower');
            
            value = Utils.getGaussianKLD(meanA, meanA, covA, covSqrtA, covSqrtA);
            
            obj.verifyEqual(value, 0);
        end
        
        
        function testGetGaussianL2DistanceScalar(obj)
            meanA    =   1;
            meanB    =  10;
            covA     = 100;
            covB     = 200;
            covSqrtA = sqrt(covA);
            covSqrtB = sqrt(covB);
            
            gA = Gaussian(meanA, covA);
            gB = Gaussian(meanB, covB);
            
            distFunc = @(x) (exp(gA.logPdf(x)) - exp(gB.logPdf(x))).^2;
            
            trueValue = sqrt(integral(distFunc, -Inf, Inf));
            
            value1 = Utils.getGaussianL2Distance(meanA, meanB, covA, covB, covSqrtA, covSqrtB);
            
            obj.verifyEqual(value1, trueValue, 'AbsTol', 1e-8);
            
            value2 = Utils.getGaussianL2Distance(meanB, meanA, covB, covA, covSqrtB, covSqrtA);
            
            obj.verifyEqual(value2, trueValue, 'AbsTol', 1e-8);
            
            obj.verifyEqual(value1, value2);
        end
        
        function testGetGaussianL2DistanceMultivariate(obj)
            meanA    = [1 10]';
            meanB    = [-15 2]';
            covA     = gallery('moler', 2);
            covB     = 0.5 * eye(2);
            covSqrtA = chol(covA, 'Lower');
            covSqrtB = chol(covB, 'Lower');
            
            gA = Gaussian(meanA, covA);
            gB = Gaussian(meanB, covB);
            
            distFunc = @(x, y) (exp(gA.logPdf([x; y])) - exp(gB.logPdf([x; y]))).^2;
            
            trueValue = sqrt(integral2(distFunc, -Inf, Inf, -Inf, Inf));
            
            value1 = Utils.getGaussianL2Distance(meanA, meanB, covA, covB, covSqrtA, covSqrtB);
            
            obj.verifyEqual(value1, trueValue, 'AbsTol', 1e-8);
            
            value2 = Utils.getGaussianL2Distance(meanB, meanA, covB, covA, covSqrtB, covSqrtA);
            
            obj.verifyEqual(value2, trueValue, 'AbsTol', 1e-8);
            
            obj.verifyEqual(value1, value2);
        end
        
        function testGetGaussianL2DistanceIdentical(obj)
            meanA    = [1 1]';
            covA     = gallery('moler', 2);
            covSqrtA = chol(covA, 'Lower');
            
            value = Utils.getGaussianL2Distance(meanA, meanA, covA, covA, covSqrtA, covSqrtA);
            
            obj.verifyEqual(value, 0);
        end
        
        
        function testRndOrthogonalMatrix(obj)
            rndMat = Utils.rndOrthogonalMatrix(4);
            
            identity = rndMat * rndMat';
            
            obj.verifyEqual(identity, eye(4), 'AbsTol', 1e-8);
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
            nominalNoise = [2.7 -0.3 -10]';
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = Utils.diffQuotientStateAndNoise(@TestUtils.stateAndNoiseFunc, ...
                                                              nominalState, nominalNoise);
            
            [trueStateJacobian, ...
             trueNoiseJacobian] = TestUtils.jacobiansStateAndNoiseFunc(nominalState, nominalNoise);
            
            [trueStateHessians, ...
             trueNoiseHessians] = TestUtils.hessiansStateAndNoiseFunc(nominalState, nominalNoise);
            
            obj.verifyEqual(stateJacobian, trueStateJacobian, 'RelTol', 1e-8);
            obj.verifyEqual(noiseJacobian, trueNoiseJacobian, 'RelTol', 1e-8);
            
            obj.verifyEqual(stateHessians, trueStateHessians, 'AbsTol', 1e-4);
            obj.verifyEqual(noiseHessians, trueNoiseHessians, 'AbsTol', 1e-4);
        end
    end
    
    methods (Static, Access = 'private')
        function values = stateFunc(states)
            values = [states(1, :).^2 .* states(2, :).^3
                      states(1, :) + sin(states(2, :))
                      exp(states(1, :)) .* sqrt(states(2, :))];
        end
        
        function stateJacobian = jacobianStateFunc(state)
            stateJacobian = [2 * state(1) * state(2)^3          3 * state(1)^2 * state(2)^2
                             1                                  cos(state(2))
                             exp(state(1)) * sqrt(state(2))     0.5 * exp(state(1)) * state(2)^(-0.5)];
        end
        
        function stateHessians = hessiansStateFunc(state)
            stateHessians(:, :, 1) = [2 * state(2)^3                6 * state(1) * state(2)^2
                                      6 * state(1) * state(2)^2     6 * state(1)^2 * state(2)];
            stateHessians(:, :, 2) = [0        0
                                      0        -sin(state(2))];
            stateHessians(:, :, 3) = [exp(state(1)) * sqrt(state(2))            0.5 * exp(state(1)) * state(2)^(-0.5)
                                      0.5 * exp(state(1)) * state(2).^(-0.5)    -1/4 * exp(state(1)) * state(2)^(-1.5)];
        end
        
        function values = stateAndNoiseFunc(state, noise)
            values = [state(1, :).^2 .* state(2, :).^3 .* noise(2, :).^2
                      state(1, :) .* noise(3, :).^3 + sin(state(2, :)) .* sqrt(noise(1, :))];
        end
        
        function [stateJacobian, noiseJacobian] = jacobiansStateAndNoiseFunc(state, noise)
            stateJacobian = [2 * state(1) * state(2)^3 * noise(2)^2    3 * state(1)^2 * state(2)^2 * noise(2)^2
                             1 * noise(3)^3                            cos(state(2)) * sqrt(noise(1))          ];
            
            noiseJacobian = [0                                         state(1)^2 * state(2)^3 * 2 * noise(2)    0
                             sin(state(2)) * 0.5 * noise(1)^(-0.5)     0                                         state(1) * 3 * noise(3)^2];
        end
        
        function [stateHessians, noiseHessians] = hessiansStateAndNoiseFunc(state, noise)
            stateHessians(:, :, 1) = [2 * state(2)^3 * noise(2)^2               6 * state(1) * state(2)^2 * noise(2)^2
                                      6 * state(1) * state(2)^2 * noise(2)^2    6 * state(1)^2 * state(2) * noise(2)^2];
            stateHessians(:, :, 2) = [0        0
                                      0        -sin(state(2)) * sqrt(noise(1))];
            
            noiseHessians(:, :, 1) = [0     0                               0
                                      0     state(1)^2 * state(2)^3 * 2     0
                                      0     0                               0];
            noiseHessians(:, :, 2) = [sin(state(2)) * -0.25 * noise(1)^(-1.5)   0   0
                                      0                                         0   0
                                      0                                         0   state(1) * 6 * noise(3)];
        end
    end
end
