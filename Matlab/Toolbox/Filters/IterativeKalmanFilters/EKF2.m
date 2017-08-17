
classdef EKF2 < IterativeKalmanFilter & SecondOrderTaylorLinearGaussianFilter
    % The second-order extended Kalman filter (EKF2).
    %
    % EKF2 Methods:
    %   EKF2                        - Class constructor.
    %   copy                        - Copy a Filter instance.
    %   copyWithName                - Copy a Filter instance and give the copy a new name/description.
    %   getName                     - Get the filter name/description.
    %   setColor                    - Set the filter color/plotting properties.
    %   getColor                    - Get the filter color/plotting properties.
    %   setState                    - Set the system state.
    %   getState                    - Get the system state.
    %   getStateDim                 - Get the dimension of the system state.
    %   getStateMeanAndCov          - Get mean and covariance matrix of the system state.
    %   predict                     - Perform a state prediction.
    %   update                      - Perform a measurement update.
    %   step                        - Perform a combined state prediction and measurement update.
    %   setStateDecompDim           - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim           - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing     - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing     - Get the post-processing method for the measurement update.
    %   setMeasGatingThreshold      - Set the measurement gating threshold.
    %   getMeasGatingThreshold      - Get the measurement gating threshold.
    %   setMaxNumIterations         - Set the maximum number of iterations that will be performed by a measurement update.
    %   getMaxNumIterations         - Get the maximum number of iterations that will be performed by a measurement update.
    %   getNumIterations            - Get number of iterations performed by the last measurement update.
    %   setConvergenceCheck         - Set a convergence check to determine if no further iterations are required.
    %   getConvergenceCheck         - Get the convergence check.
    
    % Literature:
    %   Michael Athans, Richard P. Wishner, and Anthony Bertolini,
    %   Suboptimal State Estimation for Continuous-Time Nonlinear Systems from Discrete Noisy Measurements,
    %   IEEE Transactions on Automatic Control, vol. 13, no. 5, pp. 504-514, Oct. 1968.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
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
            
            % Call superclass constructors
            obj = obj@IterativeKalmanFilter(name);
            obj = obj@SecondOrderTaylorLinearGaussianFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function setupMeasModel(obj, measModel, dimMeas)
            [noiseMean, noiseCov, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.linearizedModel = @(stateMean, stateCov) ...
                                  obj.linearizedMeasModel(measModel, dimMeas, dimNoise, ...
                                                          noiseMean, noiseCov, noiseCovSqrt, ...
                                                          stateMean, stateCov);
        end
        
        function setupAddNoiseMeasModel(obj, measModel, dimMeas)
            [addNoiseMean, addNoiseCov] = measModel.noise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.linearizedModel = @(stateMean, stateCov) ...
                                  obj.linearizedAddNoiseMeasModel(measModel, dimMeas, ...
                                                                  addNoiseMean, addNoiseCov, ...
                                                                  stateMean, stateCov);
        end
        
        function setupMixedNoiseMeasModel(obj, measModel, dimMeas)
            [noiseMean, noiseCov, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            [addNoiseMean, addNoiseCov]  = measModel.additiveNoise.getMeanAndCov();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.linearizedModel = @(stateMean, stateCov) ...
                                  obj.linearizedMixedNoiseMeasModel(measModel, dimMeas, dimNoise, ...
                                                                    noiseMean, noiseCov, noiseCovSqrt, ...
                                                                    addNoiseMean, addNoiseCov, ...
                                                                    stateMean, stateCov);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getMeasMoments(obj, priorStateMean, priorStateCov, priorStateCovSqrt)
            [measMean, H, R] = obj.linearizedModel(priorStateMean, priorStateCov);
            
            Px = H * priorStateCovSqrt;
            
            measCov           = Px * Px' + R;
            stateMeasCrossCov = priorStateCov * H';
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getMeasMomentsIteration(obj, priorStateMean, priorStateCov, priorStateCovSqrt, ...
                                                               updatedStateMean, updatedStateCov, ~)
            [h, H, R] = obj.linearizedModel(updatedStateMean, updatedStateCov);
            
            Px = H * priorStateCovSqrt;
            
            measMean          = h + H * (priorStateMean - updatedStateMean);
            measCov           = Px * Px' + R;
            stateMeasCrossCov = priorStateCov * H';
        end
    end
    
    methods (Access = 'private')
        function [h, H, R] = linearizedMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                 noiseMean, noiseCov, noiseCovSqrt, ...
                                                 stateMean, stateCov)
            [h, H, ...
             noiseJacobian, ...
             stateHessCov, ...
             noiseHessCov] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                   noiseMean, noiseCov, ...
                                                   stateMean, stateCov);
            
            Pv = noiseJacobian * noiseCovSqrt;
            R  = Pv * Pv' + stateHessCov + noiseHessCov;
        end
        
        function [h, H, R] = linearizedAddNoiseMeasModel(obj, measModel, dimMeas, ...
                                                         addNoiseMean, addNoiseCov, ...
                                                         stateMean, stateCov)
            [h, H, ...
             stateHessCov] = obj.evaluateAddNoiseMeasModel(measModel, dimMeas, ...
                                                           stateMean, stateCov);
            
            h = h + addNoiseMean;
            
            R = stateHessCov + addNoiseCov;
        end
        
        function [h, H, R] = linearizedMixedNoiseMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                           noiseMean, noiseCov, noiseCovSqrt, ...
                                                           addNoiseMean, addNoiseCov, ...
                                                           stateMean, stateCov)
            [h, H, ...
             noiseJacobian, ...
             stateHessCov, ...
             noiseHessCov] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                   noiseMean, noiseCov, ...
                                                   stateMean, stateCov);
            
            h = h + addNoiseMean;
            
            Pv = noiseJacobian * noiseCovSqrt;
            R  = Pv * Pv' + stateHessCov + noiseHessCov + addNoiseCov;
        end
    end
    
    properties (Access = 'private')
        % Function handle to the currently used model linearization method
        linearizedModel;
    end
end
