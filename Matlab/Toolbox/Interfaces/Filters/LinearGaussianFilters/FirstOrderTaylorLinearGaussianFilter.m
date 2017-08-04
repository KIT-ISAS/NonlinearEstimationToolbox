
classdef FirstOrderTaylorLinearGaussianFilter < LinearGaussianFilter
    % Abstract base class for linear Gaussian filters that are based on the
    % first-order Taylor series approximation.
    %
    % FirstOrderTaylorLinearGaussianFilter Methods:
    %   FirstOrderTaylorLinearGaussianFilter - Class constructor.
    %   copy                                 - Copy a Filter instance.
    %   copyWithName                         - Copy a Filter instance and give the copy a new name/description.
    %   getName                              - Get the filter name/description.
    %   setColor                             - Set the filter color/plotting properties.
    %   getColor                             - Get the filter color/plotting properties.
    %   setState                             - Set the system state.
    %   getState                             - Get the system state.
    %   getStateDim                          - Get the dimension of the system state.
    %   getStateMeanAndCov                   - Get mean and covariance matrix of the system state.
    %   predict                              - Perform a state prediction.
    %   update                               - Perform a measurement update.
    %   step                                 - Perform a combined state prediction and measurement update.
    %   setStateDecompDim                    - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim                    - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing          - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing          - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing              - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing              - Get the post-processing method for the measurement update.
    %   setMeasGatingThreshold               - Set the measurement gating threshold.
    %   getMeasGatingThreshold               - Get the measurement gating threshold.
    
    % Literature:
    %   Dan Simon,
    %   Optimal State Estimation,
    %   Sections 13.2,
    %   1st ed. Wiley & Sons, 2006.
    
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
        function obj = FirstOrderTaylorLinearGaussianFilter(name)
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
            %   << obj (FirstOrderTaylorLinearGaussianFilter)
            %      A new FirstOrderTaylorLinearGaussianFilter instance.
            
            % Call superclass constructor
            obj = obj@LinearGaussianFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsArbitraryNoise(obj, sysModel)
            [noiseMean, ~, noiseCovSqrt] = sysModel.noise.getMeanAndCov();
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
            A = stateJacobian * obj.stateCovSqrt;
            B = noiseJacobian * noiseCovSqrt;
            
            predictedStateCov = A * A' + B * B';
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Linearize system model around current state mean
            stateJacobian = sysModel.derivative(obj.stateMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean) + noiseMean;
            
            % Compute predicted state covariance
            A = stateJacobian * obj.stateCovSqrt;
            
            predictedStateCov = A * A' + noiseCov;
        end
        
        function setupMeasModel(obj, measModel, dimMeas)
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.linearizedModelFuncHandle = @(stateMean, stateCov, stateCovSqrt) ...
                                            obj.linearizedMeasModel(stateMean, measModel, dimMeas, ...
                                                                    dimNoise, noiseMean, noiseCovSqrt);
        end
        
        function setupAddNoiseMeasModel(obj, measModel, dimMeas)
            [addNoiseMean, addNoiseCov] = measModel.noise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.linearizedModelFuncHandle = @(stateMean, stateCov, stateCovSqrt) ...
                                            obj.linearizedAddNoiseMeasModel(stateMean, measModel, dimMeas, ...
                                                                            addNoiseMean, addNoiseCov);
        end
        
        function setupMixedNoiseMeasModel(obj, measModel, dimMeas)
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            [addNoiseMean, addNoiseCov]  = measModel.additiveNoise.getMeanAndCov();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.linearizedModelFuncHandle = @(stateMean, stateCov, stateCovSqrt) ...
                                            obj.linearizedMixedNoiseMeasModel(stateMean, measModel, dimMeas, ...
                                                                              dimNoise, noiseMean, noiseCovSqrt, ...
                                                                              addNoiseMean, addNoiseCov);
        end
        
        function [h, H, R] = linearizedModel(obj, stateMean, stateCov, stateCovSqrt)
            % Note that obj.linearizedModelFuncHandle is set by the setup*() methods,
            % which are called before the actual linear measurement update.
            [h, H, R] = obj.linearizedModelFuncHandle(stateMean, stateCov, stateCovSqrt);
        end
    end
    
    methods (Access = 'private')
        function [h, H, R] = linearizedMeasModel(obj, stateMean, measModel, dimMeas, ...
                                                 dimNoise, noiseMean, noiseCovSqrt)
            dimState = size(stateMean, 1);
            
            % Linearize measurement model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = measModel.derivative(stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            
            h = measModel.measurementEquation(stateMean, noiseMean);
            
            H = stateJacobian;
            
            A = noiseJacobian * noiseCovSqrt;
            R = A * A';
        end
        
        function [h, H, R] = linearizedAddNoiseMeasModel(obj, stateMean, measModel, dimMeas, ...
                                                         addNoiseMean, addNoiseCov)
            dimState = size(stateMean, 1);
            
            % Linearize measurement model around current state mean
            stateJacobian = measModel.derivative(stateMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            
            h = measModel.measurementEquation(stateMean) + addNoiseMean;
            
            H = stateJacobian;
            
            R = addNoiseCov;
        end
        
        function [h, H, R] = linearizedMixedNoiseMeasModel(obj, stateMean, measModel, dimMeas, ...
                                                           dimNoise, noiseMean, noiseCovSqrt, ...
                                                           addNoiseMean, addNoiseCov)
            dimState = size(stateMean, 1);
            
            % Linearize measurement model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = measModel.derivative(stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, dimMeas, dimState);
            obj.checkNoiseJacobian(noiseJacobian, dimMeas, dimNoise);
            
            h = measModel.measurementEquation(stateMean, noiseMean) + addNoiseMean;
            
            H = stateJacobian;
            
            A = noiseJacobian * noiseCovSqrt;
            R = A * A' + addNoiseCov;
        end
    end
    
    properties (Access = 'private')
        linearizedModelFuncHandle;
    end
end