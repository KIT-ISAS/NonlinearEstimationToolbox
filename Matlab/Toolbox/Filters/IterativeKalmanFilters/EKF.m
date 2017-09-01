
classdef EKF < IterativeKalmanFilter & FirstOrderTaylorLinearGaussianFilter
    % The extended Kalman filter (EKF).
    %
    % EKF Methods:
    %   EKF                         - Class constructor.
    %   copy                        - Copy a Filter instance.
    %   copyWithName                - Copy a Filter instance and give the copy a new name/description.
    %   getName                     - Get the filter name/description.
    %   setColor                    - Set the filter color/plotting properties.
    %   getColor                    - Get the filter color/plotting properties.
    %   setState                    - Set the system state.
    %   getState                    - Get the system state.
    %   setStateMeanAndCov          - Set the system state by means of mean and covariance matrix.
    %   getStateMeanAndCov          - Get mean and covariance matrix of the system state.
    %   getStateDim                 - Get the dimension of the system state.
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
    %   Dan Simon,
    %   Optimal State Estimation,
    %   Sections 13.2-13.3,
    %   1st ed. Wiley & Sons, 2006.
    
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
            
            % Call superclass constructors
            obj = obj@IterativeKalmanFilter(name);
            obj = obj@FirstOrderTaylorLinearGaussianFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function setupMeasModel(obj, measModel, dimMeas)
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.linearizedModel = @(stateMean) ...
                                  obj.linearizedMeasModel(measModel, dimMeas, dimNoise, ...
                                                          noiseMean, noiseCovSqrt, ...
                                                          stateMean);
        end
        
        function setupAddNoiseMeasModel(obj, measModel, dimMeas)
            [addNoiseMean, addNoiseCov] = measModel.noise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.linearizedModel = @(stateMean) ...
                                  obj.linearizedAddNoiseMeasModel(measModel, dimMeas, ...
                                                                  addNoiseMean, addNoiseCov, ...
                                                                  stateMean);
        end
        
        function setupMixedNoiseMeasModel(obj, measModel, dimMeas)
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            [addNoiseMean, addNoiseCov]  = measModel.additiveNoise.getMeanAndCov();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.linearizedModel = @(stateMean) ...
                                  obj.linearizedMixedNoiseMeasModel(measModel, dimMeas, dimNoise, ...
                                                                    noiseMean, noiseCovSqrt, ...
                                                                    addNoiseMean, addNoiseCov, ...
                                                                    stateMean);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getMeasMoments(obj, priorStateMean, priorStateCov, priorStateCovSqrt)
            [measMean, H, R] = obj.linearizedModel(priorStateMean);
            
            Px = H * priorStateCovSqrt;
            
            measCov           = Px * Px' + R;
            stateMeasCrossCov = priorStateCov * H';
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getMeasMomentsIteration(obj, priorStateMean, priorStateCov, priorStateCovSqrt, ...
                                                               updatedStateMean, ~, ~)
            [h, H, R] = obj.linearizedModel(updatedStateMean);
            
            Px = H * priorStateCovSqrt;
            
            measMean          = h + H * (priorStateMean - updatedStateMean);
            measCov           = Px * Px' + R;
            stateMeasCrossCov = priorStateCov * H';
        end
    end
    
    methods (Access = 'private')
        function [h, H, R] = linearizedMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                 noiseMean, noiseCovSqrt, ...
                                                 stateMean)
            [h, H, noiseJacobian] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                          noiseMean, stateMean);
            
            Pv = noiseJacobian * noiseCovSqrt;
            R  = Pv * Pv';
        end
        
        function [h, H, R] = linearizedAddNoiseMeasModel(obj, measModel, dimMeas, ...
                                                         addNoiseMean, addNoiseCov, ...
                                                         stateMean)
            [h, H] = obj.evaluateAddNoiseMeasModel(measModel, dimMeas, stateMean);
            
            h = h + addNoiseMean;
            
            R = addNoiseCov;
        end
        
        function [h, H, R] = linearizedMixedNoiseMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                           noiseMean, noiseCovSqrt, ...
                                                           addNoiseMean, addNoiseCov, ...
                                                           stateMean)
            [h, H, noiseJacobian] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                          noiseMean, stateMean);
            
            h = h + addNoiseMean;
            
            Pv = noiseJacobian * noiseCovSqrt;
            R  = Pv * Pv' + addNoiseCov;
        end
    end
    
    properties (Access = 'private')
        % Function handle to the currently used model linearization method
        linearizedModel;
    end
end
