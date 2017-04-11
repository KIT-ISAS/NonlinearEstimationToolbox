
classdef SOTaylorBasedJointlyGaussianPrediction < GaussianFilter
    % Abstract base class for Gaussian filters that use a second-order Taylor based prediction,
    % where the joint density of state and noise is assumed to be Gaussian.
    %
    % SOTaylorBasedJointlyGaussianPrediction Methods:
    %   SOTaylorBasedJointlyGaussianPrediction - Class constructor.
    %   copy                                   - Copy a Filter instance.
    %   copyWithName                           - Copy a Filter instance and give the copy a new name / description.
    %   getName                                - Get the filter name / description.
    %   setColor                               - Set the filter color / plotting properties.
    %   getColor                               - Get the current filter color / plotting properties.
    %   setState                               - Set the system state.
    %   getState                               - Get the current system state.
    %   getStateDim                            - Get the dimension of the current system state.
    %   predict                                - Perform a time update (prediction step).
    %   update                                 - Perform a measurement update (filter step) using the given measurement(s).
    %   step                                   - Perform a combined time and measurement update.
    %   getPointEstimate                       - Get a point estimate of the current system state.
    %   setStateDecompDim                      - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim                      - Get the dimension of the unobservable part of the system state.
    
    % Literature:
    %   Michael Athans, Richard P. Wishner, and Anthony Bertolini,
    %   Suboptimal State Estimation for Continuous-Time Nonlinear Systems from Discrete Noisy Measurements,
    %   IEEE Transactions on Automatic Control, Vol. 13, No. 5, Oct. 1968, pp. 504-514.
    
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
    
    methods
        function obj = SOTaylorBasedJointlyGaussianPrediction(name)
            % Class constructor.
            %
            % Parameters:
            %   >> name (Char)
            %      An appropriate filter name / description of the implemented
            %      filter. The Filter subclass should set this during its
            %      construction to a meaningful default value (e.g., 'EKF'),
            %      or the user should specify an appropriate name (e.g., 
            %      'PF (10k Particles)').
            %
            % Returns:
            %   << obj (SOTaylorBasedJointlyGaussianPrediction)
            %      A new SOTaylorBasedJointlyGaussianPrediction instance.
            
            % Call superclass constructor
            obj = obj@GaussianFilter(name);
        end
    end
    
    methods (Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsArbitraryNoise(obj, sysModel)
            [noiseMean, noiseCov, noiseCovSqrt] = sysModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            % Compute system model derivatives around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = sysModel.derivative(obj.stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            obj.checkStateHessians(stateHessians, obj.dimState, obj.dimState);
            
            obj.checkNoiseJacobian(noiseJacobian, obj.dimState, dimNoise);
            obj.checkNoiseHessians(noiseHessians, obj.dimState, dimNoise);
            
            [stateHessMean, ...
             stateHessCov, ...
             noiseHessMean, ...
             noiseHessCov] = obj.getHessianMomentsStateAndNoise(obj.dimState, stateHessians, obj.stateCov, ...
                                                                dimNoise, noiseHessians, noiseCov, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean, noiseMean) + ...
                                 stateHessMean + noiseHessMean;
            
            % Compute predicted state covariance
            A = stateJacobian * obj.stateCovSqrt;
            B = noiseJacobian * noiseCovSqrt;
            
            predictedStateCov = A * A' + stateHessCov + B * B' + noiseHessCov;
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Compute system model derivatives around current state mean
            [stateJacobian, stateHessians] = sysModel.derivative(obj.stateMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            obj.checkStateHessians(stateHessians, obj.dimState, obj.dimState);
            
            [stateHessMean, ...
             stateHessCov] = obj.getHessianMomentsState(obj.dimState, stateHessians, ...
                                                        obj.stateCov, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean) + stateHessMean + noiseMean;
            
            % Compute predicted state covariance
            A = stateJacobian * obj.stateCovSqrt;
            
            predictedStateCov = A * A' + stateHessCov + noiseCov;
        end
        
        function [stateHessMean, ...
                  stateHessCov] = getHessianMomentsState(~, dimState, stateHessians, stateCov, dimOutput)
            stateHessProd = nan(dimState, dimState, dimState);
            
            stateHessMean = nan(dimOutput, 1);
            
            for i = 1:dimOutput
                stateHessProd(:, :, i) = stateHessians(:, :, i) * stateCov;
                stateHessMean(i)       = trace(stateHessProd(:, :, i));
            end
            
            stateHessCov = nan(dimOutput, dimOutput);
            
            for i = 1:dimOutput
                mat                = stateHessProd(:, :, i) .* stateHessProd(:, :, i)';
                stateHessCov(i, i) = sum(mat(:));        % = trace(stateHessProd(:, :, i)^2)
                
                for j = (i + 1):dimOutput
                    mat                = stateHessProd(:, :, i) .* stateHessProd(:, :, j)';
                    stateHessCov(i, j) = sum(mat(:));    % = trace(stateHessProd(:, :, i) * stateHessProd(:, :, j))
                    stateHessCov(j, i) = stateHessCov(i, j);
                end
            end
            
            stateHessMean = 0.5 * stateHessMean;
            stateHessCov  = 0.5 * stateHessCov;
        end
        
        function [stateHessMean, ...
                  stateHessCov, ...
                  noiseHessMean, ...
                  noiseHessCov] = getHessianMomentsStateAndNoise(~, dimState, stateHessians, stateCov, ...
                                                                 dimNoise, noiseHessians, noiseCov, dimOutput)
            stateHessProd = nan(dimState, dimState, dimState);
            noiseHessProd = nan(dimNoise, dimNoise, dimState);
            
            stateHessMean = nan(dimOutput, 1);
            noiseHessMean = nan(dimOutput, 1);
            
            for i = 1:dimOutput
                stateHessProd(:, :, i) = stateHessians(:, :, i) * stateCov;
                stateHessMean(i)       = trace(stateHessProd(:, :, i));
                
                noiseHessProd(:, :, i) = noiseHessians(:, :, i) * noiseCov;
                noiseHessMean(i)       = trace(noiseHessProd(:, :, i));
            end
            
            stateHessCov = nan(dimOutput, dimOutput);
            noiseHessCov = nan(dimOutput, dimOutput);
            
            for i = 1:dimOutput
                mat                = stateHessProd(:, :, i) .* stateHessProd(:, :, i)';
                stateHessCov(i, i) = sum(mat(:));           % = trace(stateHessProd(:, :, i)^2)
                
                mat                = noiseHessProd(:, :, i) .* noiseHessProd(:, :, i)';
                noiseHessCov(i, i) = sum(mat(:));           % = trace(noiseHessProd(:, :, i)^2)
                
                for j = (i + 1):dimOutput
                    mat                = stateHessProd(:, :, i) .* stateHessProd(:, :, j)';
                    stateHessCov(i, j) = sum(mat(:));       % = trace(stateHessProd(:, :, i) * stateHessProd(:, :, j))
                    stateHessCov(j, i) = stateHessCov(i, j);
                    
                    mat                = noiseHessProd(:, :, i) .* noiseHessProd(:, :, j)';
                    noiseHessCov(i, j) = sum(mat(:));       % = trace(noiseHessProd(:, :, i) * noiseHessProd(:, :, j))
                    noiseHessCov(j, i) = noiseHessCov(i, j);
                end
            end
            
            stateHessMean = 0.5 * stateHessMean;
            stateHessCov  = 0.5 * stateHessCov;
            
            noiseHessMean = 0.5 * noiseHessMean;
            noiseHessCov  = 0.5 * noiseHessCov;
        end
    end
end
