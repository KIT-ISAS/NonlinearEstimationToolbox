
classdef EKF < KF
    % The Extended Kalman Filter (EKF).
    %
    % EKF Methods:
    %   EKF                            - Class constructor.
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
    %   Dan Simon,
    %   Optimal State Estimation,
    %   1st ed. Wiley & Sons, 2006.
    
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
    
    methods
        function obj = EKF(name)
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
            %      Default name: 'EKF'.
            %
            % Returns:
            %   << obj (EKF)
            %      A new EKF instance.
            
            if nargin < 1
                name = 'EKF';
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
            
            % Linearize system model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = sysModel.derivative(obj.stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            obj.checkNoiseJacobian(noiseJacobian, obj.dimState, dimNoise);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean, noiseMean);
            
            % Compute predicted state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + ...
                                noiseJacobian * noiseCov * noiseJacobian';
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Linearize system model around current state mean
            stateJacobian = sysModel.derivative(obj.stateMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean) + noiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + noiseCov;
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
                  stateMeasCrossCov] = momentFuncArbitraryNoise(obj, priorMean, priorCov, ~, iterMean, ~, ~, ...
                                                                measModel, dimNoise, dimMeas, numMeas, noiseMean, noiseCov)
            dimState = size(iterMean, 1);
            
            % Linearize measurement model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = measModel.derivative(iterMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterMean, noiseMean) + ...
                       stateJacobian * (priorMean - iterMean);
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * priorCov * stateJacobian';
            matDiag = noiseJacobian * noiseCov * noiseJacobian';
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncAdditiveNoise(obj, priorMean, priorCov, ~, iterMean, ~, ~, ...
                                                               measModel, dimMeas, numMeas, noiseMean, noiseCov)
            dimState = size(iterMean, 1);
            
            % Linearize measurement model around current state mean
            stateJacobian = measModel.derivative(iterMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterMean) + noiseMean + ...
                       stateJacobian * (priorMean - iterMean);
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * priorCov * stateJacobian';
            
            measCov = Utils.baseBlockDiag(matBase, noiseCov, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMixedNoise(obj, priorMean, priorCov, ~, iterMean, ~, ~, ...
                                                            measModel, dimNoise, dimMeas, numMeas, addNoiseMean, addNoiseCov, noiseMean, noiseCov)
            dimState = size(iterMean, 1);
            
            % Linearize measurement model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = measModel.derivative(iterMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterMean, noiseMean) + addNoiseMean + ...
                       stateJacobian * (priorMean - iterMean);
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * priorCov * stateJacobian';
            matDiag = noiseJacobian * noiseCov * noiseJacobian' + addNoiseCov;
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
    end
end
