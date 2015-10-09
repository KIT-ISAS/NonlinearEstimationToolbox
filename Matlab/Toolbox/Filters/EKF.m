
classdef EKF < KF
    % The Extended Kalman Filter (EKF).
    %
    % EKF Methods:
    %   EKF                        - Class constructor.
    %   getName                    - Get the filter name / description.
    %   setColor                   - Set the filter color / plotting properties.
    %   getColor                   - Get the current filter color / plotting properties.
    %   setState                   - Set the system state.
    %   getState                   - Get the current system state.
    %   getStateDim                - Get the dimension of the current system state.
    %   predict                    - Perform a time update (prediction step).
    %   update                     - Perform a measurement update (filter step) using the given measurement(s).
    %   step                       - Perform a combined time and measurement update.
    %   getPointEstimate           - Get a point estimate of the current system state.
    %   setMaxNumIterations        - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations        - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold - Get the current measurement validation threshold.
    %   getLastUpdateData          - Get information from the last performed measurement update.
    
    % Literature:
    %   Dan Simon, Optimal State Estimation, 1st ed. Wiley & Sons, 2006
    
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
            %   << obj (AnalyticKF)
            %      A new AnalyticKF instance.
            
            if nargin < 1
                name = 'EKF';
            end
            
            % Superclass constructor
            obj = obj@KF(name);
        end 
    end
    
    methods (Access = 'protected')
        function performPrediction(obj, sysModel)
            if Checks.isClass(sysModel, 'LinearSystemModel')
                obj.predictAnalytic(sysModel);
            elseif Checks.isClass(sysModel, 'SystemModel')
                obj.predictArbitraryNoise(sysModel);
            elseif Checks.isClass(sysModel, 'AdditiveNoiseSystemModel')
                obj.predictAdditiveNoise(sysModel);
            elseif Checks.isClass(sysModel, 'MixedNoiseSystemModel')
                obj.predictMixedNoise(sysModel);
            else
                obj.errorSysModel('Linear system model', ...
                                  'System model', ...
                                  'Additive noise system model', ...
                                  'Mixed noise system model');
            end
        end
        
        function predictArbitraryNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            % Linearize system model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = sysModel.derivative(obj.stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState);
            obj.checkNoiseJacobian(noiseJacobian, obj.dimState, dimNoise);
            
            % Predict state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean, noiseMean);
            
            % Predict state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + ...
                                noiseJacobian * noiseCov * noiseJacobian';
            
            obj.checkAndSavePrediction(predictedStateMean, predictedStateCov);
        end
        
        function predictAdditiveNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Linearize system model around current state mean
            stateJacobian = sysModel.derivative(obj.stateMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, obj.dimState);
            
            % Predict state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean) + noiseMean;
            
            % Predict state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + noiseCov;
            
            obj.checkAndSavePrediction(predictedStateMean, predictedStateCov);
        end
        
        function predictMixedNoise(obj, sysModel)
            [noiseMean, noiseCov]       = sysModel.noise.getMeanAndCovariance();
            [addNoiseMean, addNoiseCov] = sysModel.additiveNoise.getMeanAndCovariance();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimAddNoise);
            
            % Linearize system model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = sysModel.derivative(obj.stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState);
            obj.checkNoiseJacobian(noiseJacobian, obj.dimState, dimNoise);
            
            % Predict state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean, noiseMean) + addNoiseMean;
            
            % Predict state covariance
            predictedStateCov = stateJacobian * obj.stateCov * stateJacobian' + ...
                                noiseJacobian * noiseCov * noiseJacobian' + ...
                                addNoiseCov;
            
            obj.checkAndSavePrediction(predictedStateMean, predictedStateCov);
        end
        
        function performUpdate(obj, measModel, measurements)
            if Checks.isClass(measModel, 'LinearMeasurementModel')
                obj.updateAnalytic(measModel, measurements);
            elseif Checks.isClass(measModel, 'MeasurementModel')
                obj.updateArbitraryNoise(measModel, measurements);
            elseif Checks.isClass(measModel, 'AdditiveNoiseMeasurementModel')
                obj.updateAdditiveNoise(measModel, measurements);
            elseif Checks.isClass(measModel, 'MixedNoiseMeasurementModel')
                obj.updateMixedNoise(measModel, measurements);
            else
                obj.errorMeasModel('Linear measurement model', ...
                                   'Measurement model', ...
                                   'Additive noise measurement model', ...
                                   'Mixed noise measurement model');
            end
        end
        
        function updateArbitraryNoise(obj, measModel, measurements)
            [dimMeas, numMeas]    = size(measurements);
            [noiseMean, noiseCov] = measModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            % Perform state update
            obj.kalmanUpdate(measModel, measurements, @obj.momentFuncArbitraryNoise, ...
                             dimNoise, dimMeas, numMeas, noiseMean, noiseCov);
        end
        
        function updateAdditiveNoise(obj, measModel, measurements)
            [dimMeas, numMeas]    = size(measurements);
            [noiseMean, noiseCov] = measModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimNoise);
            
            % Perform state update
            obj.kalmanUpdate(measModel, measurements, @obj.momentFuncAdditiveNoise, ...
                             dimMeas, numMeas, noiseMean, noiseCov);
        end
        
        function updateMixedNoise(obj, measModel, measurements)
            [dimMeas, numMeas]          = size(measurements);
            [noiseMean, noiseCov]       = measModel.noise.getMeanAndCovariance();
            [addNoiseMean, addNoiseCov] = measModel.additiveNoise.getMeanAndCovariance();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            % Perform state update
            obj.kalmanUpdate(measModel, measurements, @obj.momentFuncMixedNoise, ...
                             dimNoise, dimMeas, numMeas, addNoiseMean, addNoiseCov, noiseMean, noiseCov);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncArbitraryNoise(obj, measModel, ~, iterStateMean, ~, ~, ...
                                                                dimNoise, dimMeas, numMeas, noiseMean, noiseCov)
            % Linearize measurement model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = measModel.derivative(iterStateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas);
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterStateMean, noiseMean) + ...
                       stateJacobian * (obj.stateMean - iterStateMean);
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * obj.stateCov * stateJacobian';
            matDiag = noiseJacobian * noiseCov * noiseJacobian';
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = obj.stateCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncAdditiveNoise(obj, measModel, ~, iterStateMean, ~, ~, ...
                                                               dimMeas, numMeas, noiseMean, noiseCov)
            % Linearize measurement model around current state mean
            stateJacobian = measModel.derivative(iterStateMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, dimMeas);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterStateMean) + noiseMean + ...
                       stateJacobian * (obj.stateMean - iterStateMean);
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * obj.stateCov * stateJacobian';
            
            measCov = Utils.baseBlockDiag(matBase, noiseCov, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = obj.stateCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMixedNoise(obj, measModel, ~, iterStateMean, ~, ~, ...
                                                            dimNoise, dimMeas, numMeas, addNoiseMean, addNoiseCov, noiseMean, noiseCov)
            % Linearize measurement model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = measModel.derivative(iterStateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas);
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            
            % Compute measurement mean
            measMean = measModel.measurementEquation(iterStateMean, noiseMean) + addNoiseMean + ...
                       stateJacobian * (obj.stateMean - iterStateMean);
            
            measMean = repmat(measMean, numMeas, 1);
            
            % Compute measurement covariance
            matBase = stateJacobian * obj.stateCov * stateJacobian';
            matDiag = noiseJacobian * noiseCov * noiseJacobian' + addNoiseCov;
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = obj.stateCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function checkStateJacobian(obj, stateJacobian, dimOutput)
            if ~Checks.isMat(stateJacobian, dimOutput, obj.dimState)
                obj.error('InvalidStateJacobian', ...
                          'State Jacobian must be a matrix of dimension %dx%d.', ...
                          dimOutput, obj.dimState);
            end
        end
        
        function checkNoiseJacobian(obj, noiseJacobian, dimOutput, dimNoise)
            if ~Checks.isMat(noiseJacobian, dimOutput, dimNoise)
                obj.error('InvalidNoiseJacobian', ...
                          'Noise Jacobian has to be a matrix of dimension %dx%d.', ...
                          dimOutput, dimNoise);
            end
        end
    end
end
