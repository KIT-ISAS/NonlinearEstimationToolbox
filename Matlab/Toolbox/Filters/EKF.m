
classdef EKF < KF & FOTaylorBasedJointlyGaussianPrediction
    % The Extended Kalman Filter (EKF).
    %
    % EKF Methods:
    %   EKF                        - Class constructor.
    %   copy                       - Copy a Filter instance.
    %   copyWithName               - Copy a Filter instance and give the copy a new name / description.
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
    %   setStateDecompDim          - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim          - Get the dimension of the unobservable part of the system state.
    %   setMaxNumIterations        - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations        - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold - Get the current measurement validation threshold.
    %   getLastUpdateData          - Get information from the last performed measurement update.
    
    % Literature:
    %   Dan Simon,
    %   Optimal State Estimation,
    %   Sections 13.2-13.3,
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
            
            % Superclass constructors
            obj = obj@KF(name);
            obj = obj@FOTaylorBasedJointlyGaussianPrediction(name);
        end 
    end
    
    methods (Access = 'protected')
        function momentFunc = getMomentFuncArbitraryNoise(obj, measModel, measurements)
            [dimMeas, numMeas]    = size(measurements);
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            momentFunc = @(priorMean, priorCov, priorCovSqrt, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncArbitraryNoise(priorMean, priorCov, priorCovSqrt, ...
                                                      iterNum, iterMean, iterCov, iterCovSqrt, ...
                                                      measModel, dimNoise, dimMeas, numMeas, noiseMean, noiseCovSqrt);
        end
        
        function momentFunc = getMomentFuncAdditiveNoise(obj, measModel, measurements)
            [dimMeas, numMeas]    = size(measurements);
            [noiseMean, noiseCov] = measModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimNoise);
            
            momentFunc = @(priorMean, priorCov, priorCovSqrt, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncAdditiveNoise(priorMean, priorCov, priorCovSqrt, ...
                                                     iterNum, iterMean, iterCov, iterCovSqrt, ...
                                                     measModel, dimMeas, numMeas, noiseMean, noiseCov);
        end
        
        function momentFunc = getMomentFuncMixedNoise(obj, measModel, measurements)
            [dimMeas, numMeas]           = size(measurements);
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            [addNoiseMean, addNoiseCov]  = measModel.additiveNoise.getMeanAndCov();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            momentFunc = @(priorMean, priorCov, priorCovSqrt, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncMixedNoise(priorMean, priorCov, priorCovSqrt, ...
                                                  iterNum, iterMean, iterCov, iterCovSqrt, ...
                                                  measModel, dimNoise, dimMeas, numMeas, addNoiseMean, ...
                                                  addNoiseCov, noiseMean, noiseCovSqrt);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncArbitraryNoise(obj, priorMean, priorCov, priorCovSqrt, ...
                                                                ~, iterMean, ~, ~, ...
                                                                measModel, dimNoise, dimMeas, numMeas, noiseMean, noiseCovSqrt)
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
            A = stateJacobian * priorCovSqrt;
            B = noiseJacobian * noiseCovSqrt;
            
            matBase = A * A';
            matDiag = B * B';
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncAdditiveNoise(obj, priorMean, priorCov, priorCovSqrt, ...
                                                               ~, iterMean, ~, ~, ...
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
            A = stateJacobian * priorCovSqrt;
            
            matBase = A * A';
            
            measCov = Utils.baseBlockDiag(matBase, noiseCov, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMixedNoise(obj, priorMean, priorCov, priorCovSqrt, ...
                                                            ~, iterMean, ~, ~, ...
                                                            measModel, dimNoise, dimMeas, numMeas, addNoiseMean, addNoiseCov, noiseMean, noiseCovSqrt)
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
            A = stateJacobian * priorCovSqrt;
            B = noiseJacobian * noiseCovSqrt;
            
            matBase = A * A';
            matDiag = B * B' + addNoiseCov;
            
            measCov = Utils.baseBlockDiag(matBase, matDiag, numMeas);
            
            % Compute state measurement cross-covariance
            stateMeasCrossCov = priorCov * stateJacobian';
            
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
    end
end
