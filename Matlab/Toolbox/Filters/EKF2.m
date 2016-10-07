
classdef EKF2 < KF
    % The Second-Order Extended Kalman Filter (EKF).
    %
    % EKF2 Methods:
    %   EKF2                           - Class constructor.
    %   getName                        - Get the filter name / description.
    %   setColor                       - Set the filter color / plotting properties.
    %   getColor                       - Get the current filter color / plotting properties.
    %   setState                       - Set the system state.
    %   getState                       - Get the current system state.
    %   getStateDim                    - Get the dimension of the current system state.
    %   predict                        - Perform a time update (prediction step).
    %   update                         - Perform a measurement update (filter step) using the given measurement(s).
    %   step                           - Perform a combined time and measurement update.
    %   getPointEstimate               - Get a point estimate of the current system state.
    %   setUseAnalyticSystemModel      - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel      - Get the current use of analytic moment calculation during a prediction.
    %   setStateDecompDim              - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim              - Get the dimension of the unobservable part of the system state.
    %   setUseAnalyticMeasurementModel - Enable or disable the use of analytic moment calculation during a filter step.
    %   getUseAnalyticMeasurementModel - Get the current use of analytic moment calculation during a filter step.
    %   setMaxNumIterations            - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations            - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold     - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold     - Get the current measurement validation threshold.
    %   getLastUpdateData              - Get information from the last performed measurement update.
    
    % Literature:
    %   Michael Athans, Richard P. Wishner, and Anthony Bertolini,
    %   Suboptimal State Estimation for Continuous-Time Nonlinear Systems from Discrete Noisy Measurements,
    %   IEEE Transactions on Automatic Control, Vol. 13, No. 5, Oct. 1968, pp. 504-514.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2016  Jannik Steinbring <jannik.steinbring@kit.edu>
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
        function obj = EKF2(name)
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
            %      Default name: 'EKF2'.
            %
            % Returns:
            %   << obj (EKF2)
            %      A new EKF2 instance.
            
            if nargin < 1
                name = 'EKF2';
            end
            
            % Superclass constructor
            obj = obj@KF(name);
        end
    end
    
    methods (Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsArbitraryNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
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
            
            [hessMeanState, ...
             hessCovState, ...
             hessMeanNoise, ...
             hessCovNoise] = EKF2.getHessianMomentsStateAndNoise(obj.dimState, stateHessians, obj.stateCov, ...
                                                                 dimNoise, noiseHessians, noiseCov, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean, noiseMean) + ...
                                 hessMeanState + hessMeanNoise;
            
            % Compute predicted state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + hessCovState + ...
                                noiseJacobian * noiseCov * noiseJacobian' + hessCovNoise;
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Compute system model derivatives around current state mean
            [stateJacobian, stateHessians] = sysModel.derivative(obj.stateMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            obj.checkStateHessians(stateHessians, obj.dimState, obj.dimState);
            
            [hessMean, hessCov] = EKF2.getHessianMomentsState(obj.dimState, stateHessians, ...
                                                              obj.stateCov, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean) + hessMean + noiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + hessCov + noiseCov;
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsMixedNoise(obj, sysModel)
            [addNoiseMean, addNoiseCov] = sysModel.additiveNoise.getMeanAndCovariance();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimAddNoise);
            
            [mean, cov] = obj.predictedMomentsArbitraryNoise(sysModel);
            
            % Compute predicted state mean
            predictedStateMean = mean + addNoiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + addNoiseCov;
        end
        
        function momentFunc = getMomentFuncArbitraryNoise(obj, measModel, measurements)
            [dimMeas, numMeas]    = size(measurements);
            [noiseMean, noiseCov] = measModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            momentFunc = @(priorMean, priorCov, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncArbitraryNoise(priorMean, priorCov, iterNum, iterMean, iterCov, iterCovSqrt, ...
                                                      measModel, dimNoise, dimMeas, numMeas, noiseMean, noiseCov);
        end
        
        function momentFunc = getMomentFuncAdditiveNoise(obj, measModel, measurements)
            [dimMeas, numMeas]    = size(measurements);
            [noiseMean, noiseCov] = measModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimNoise);
            
            momentFunc = @(priorMean, priorCov, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncAdditiveNoise(priorMean, priorCov, iterNum, iterMean, iterCov, iterCovSqrt, ...
                                                     measModel, dimMeas, numMeas, noiseMean, noiseCov);
        end
        
        function momentFunc = getMomentFuncMixedNoise(obj, measModel, measurements)
            [dimMeas, numMeas]          = size(measurements);
            [noiseMean, noiseCov]       = measModel.noise.getMeanAndCovariance();
            [addNoiseMean, addNoiseCov] = measModel.additiveNoise.getMeanAndCovariance();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            momentFunc = @(priorMean, priorCov, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncMixedNoise(priorMean, priorCov, iterNum, iterMean, iterCov, iterCovSqrt, ...
                                                  measModel, dimNoise, dimMeas, numMeas, addNoiseMean, ...
                                                  addNoiseCov, noiseMean, noiseCov);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncArbitraryNoise(obj, priorMean, priorCov, ~, iterMean, iterCov, ~, ...
                                                                measModel, dimNoise, dimMeas, numMeas, noiseMean, noiseCov)
            dimState = size(iterMean, 1);
            
            % Compute measurement model derivatives around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = measModel.derivative(iterMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkStateHessians(stateHessians, dimMeas, dimState);
            
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            obj.checkNoiseHessians(noiseHessians, dimMeas, dimNoise);
            
            [hessMeanState, ...
             hessCovState, ...
             hessMeanNoise, ...
             hessCovNoise] = EKF2.getHessianMomentsStateAndNoise(dimState, stateHessians, iterCov, ...
                                                                 dimNoise, noiseHessians, noiseCov, dimMeas);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterMean, noiseMean) + ...
                       stateJacobian * (priorMean - iterMean) + ...
                       hessMeanState + hessMeanNoise;
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * priorCov * stateJacobian' + hessCovState;
            matDiag = noiseJacobian * noiseCov * noiseJacobian' + hessCovNoise;
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncAdditiveNoise(obj, priorMean, priorCov, ~, iterMean, iterCov, ~, ...
                                                               measModel, dimMeas, numMeas, noiseMean, noiseCov)
            dimState = size(iterMean, 1);
            
            % Compute measurement model derivatives around current state mean
            [stateJacobian, stateHessians] = measModel.derivative(iterMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkStateHessians(stateHessians, dimMeas, dimState);
            
            [hessMean, hessCov] = EKF2.getHessianMomentsState(dimState, stateHessians, iterCov, dimMeas);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterMean) + ...
                       stateJacobian * (priorMean - iterMean) + ...
                       hessMean + noiseMean;
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * priorCov * stateJacobian' + hessCov;
            
            measCov = Utils.baseBlockDiag(matBase, noiseCov, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMixedNoise(obj, priorMean, priorCov, ~, iterMean, iterCov, ~, ...
                                                            measModel, dimNoise, dimMeas, numMeas, addNoiseMean, addNoiseCov, noiseMean, noiseCov)
            dimState = size(iterMean, 1);
            
            % Compute measurement model derivatives around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = measModel.derivative(iterMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkStateHessians(stateHessians, dimMeas, dimState);
            
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            obj.checkNoiseHessians(noiseHessians, dimMeas, dimNoise);
            
            [hessMeanState, ...
             hessCovState, ...
             hessMeanNoise, ...
             hessCovNoise] = EKF2.getHessianMomentsStateAndNoise(dimState, stateHessians, iterCov, ...
                                                                 dimNoise, noiseHessians, noiseCov, dimMeas);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterMean, noiseMean) + ...
                       stateJacobian * (priorMean - iterMean) + ...
                       hessMeanState + hessMeanNoise + addNoiseMean;
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * priorCov * stateJacobian' + hessCovState;
            matDiag = noiseJacobian * noiseCov * noiseJacobian' + hessCovNoise + addNoiseCov;
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
    end
    
    methods (Static, Access = 'private')
        function [hessMean, hessCov] = getHessianMomentsState(dimState, stateHessians, stateCov, dimOutput)
            % Compute predicted state mean
            hessProd = nan(dimState, dimState, dimState);
            hessMean = nan(dimOutput, 1);
            
            for i = 1:dimOutput
                hessProd(:, :, i) = stateHessians(:, :, i) * stateCov;
                hessMean(i)       = trace(hessProd(:, :, i));
            end
            
            hessCov = nan(dimOutput, dimOutput);
            
            for i = 1:dimOutput
                mat           = hessProd(:, :, i) .* hessProd(:, :, i)';
                hessCov(i, i) = sum(mat(:));        % = trace(hessProd(:, :, i)^2)
                
                for j = (i + 1):dimOutput
                    mat           = hessProd(:, :, i) .* hessProd(:, :, j)';
                    hessCov(i, j) = sum(mat(:));    % = trace(hessProd(:, :, i) * hessProd(:, :, j))
                    hessCov(j, i) = hessCov(i, j);
                end
            end
            
            hessMean = 0.5 * hessMean;
            hessCov  = 0.5 * hessCov;
        end
        
        function [hessMeanState, ...
                  hessCovState, ...
                  hessMeanNoise, ...
                  hessCovNoise] = getHessianMomentsStateAndNoise(dimState, stateHessians, stateCov, ...
                                                                 dimNoise, noiseHessians, noiseCov, dimOutput)
            % Compute predicted state mean
            hessProdState = nan(dimState, dimState, dimState);
            hessProdNoise = nan(dimNoise, dimNoise, dimState);
            hessMeanState = nan(dimOutput, 1);
            hessMeanNoise = nan(dimOutput, 1);
            
            for i = 1:dimOutput
                hessProdState(:, :, i) = stateHessians(:, :, i) * stateCov;
                hessMeanState(i)       = trace(hessProdState(:, :, i));
                
                hessProdNoise(:, :, i) = noiseHessians(:, :, i) * noiseCov;
                hessMeanNoise(i)       = trace(hessProdNoise(:, :, i));
            end
            
            % Compute predicted state covariance
            hessCovState = nan(dimOutput, dimOutput);
            hessCovNoise = nan(dimOutput, dimOutput);
            
            for i = 1:dimOutput
                mat                = hessProdState(:, :, i) .* hessProdState(:, :, i)';
                hessCovState(i, i) = sum(mat(:));           % = trace(hessProdState(:, :, i)^2)
                
                mat                = hessProdNoise(:, :, i) .* hessProdNoise(:, :, i)';
                hessCovNoise(i, i) = sum(mat(:));           % = trace(hessProdNoise(:, :, i)^2)
                
                for j = (i + 1):dimOutput
                    mat                = hessProdState(:, :, i) .* hessProdState(:, :, j)';
                    hessCovState(i, j) = sum(mat(:));       % = trace(hessProdState(:, :, i) * hessProdState(:, :, j))
                    hessCovState(j, i) = hessCovState(i, j);
                    
                    mat                = hessProdNoise(:, :, i) .* hessProdNoise(:, :, j)';
                    hessCovNoise(i, j) = sum(mat(:));       % = trace(hessProdNoise(:, :, i) * hessProdNoise(:, :, j))
                    hessCovNoise(j, i) = hessCovNoise(i, j);
                end
            end
            
            hessMeanState = 0.5 * hessMeanState;
            hessMeanNoise = 0.5 * hessMeanNoise;
            hessCovState  = 0.5 * hessCovState;
            hessCovNoise  = 0.5 * hessCovNoise;
        end
    end
end
