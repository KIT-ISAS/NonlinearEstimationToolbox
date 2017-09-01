
classdef ERUF < RecursiveUpdateFilter & FirstOrderTaylorLinearGaussianFilter
    % The extended recursive update filter (ERUF).
    %
    % ERUF Methods:
    %   ERUF                        - Class constructor.
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
    %   setNumRecursionSteps        - Set the number of recursion steps that are performed by a measurement update.
    %   getNumRecursionSteps        - Get the number of recursion steps that are performed by a measurement update.
    
    % Literature:
    %   Renato Zanetti,
    %   Recursive Update Filtering for Nonlinear Estimation,
    %   IEEE Transactions on Automatic Control, vol. 57, no. 6, pp. 1481-1490, Jun. 2012.
    
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
        function obj = ERUF(name)
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
            %      Default name: 'ERUF'.
            %
            % Returns:
            %   << obj (ERUF)
            %      A new ERUF instance.
            
            if nargin < 1
                name = 'ERUF';
            end
            
            % Call superclass constructors
            obj = obj@RecursiveUpdateFilter(name);
            obj = obj@FirstOrderTaylorLinearGaussianFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function setupMeasModel(obj, measModel, dimMeas)
            [noiseMean, noiseCov, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.momentFuncHandle = @(stateMean, stateCov, stateCovSqrt) ...
                                   obj.momentFuncMeasModel(measModel, dimMeas, dimNoise, ...
                                                           noiseMean, noiseCov, noiseCovSqrt, ...
                                                           stateMean, stateCov, stateCovSqrt);
            
            obj.momentFuncCorrHandle = @(stateMean, stateCov, stateCovSqrt, ...
                                         stateNoiseCrossCov) ...
                                       obj.momentFuncCorrMeasModel(measModel, dimMeas, dimNoise, ...
                                                                   noiseMean, noiseCov, noiseCovSqrt, ...
                                                                   stateMean, stateCov, stateCovSqrt, ...
                                                                   stateNoiseCrossCov);
        end
        
        function setupAddNoiseMeasModel(obj, measModel, dimMeas)
            [addNoiseMean, addNoiseCov] = measModel.noise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.momentFuncHandle = @(stateMean, stateCov, stateCovSqrt) ...
                                   obj.momentFuncAddNoiseMeasModel(measModel, dimMeas, ...
                                                                   addNoiseMean, addNoiseCov, ...
                                                                   stateMean, stateCov, stateCovSqrt);
            
            obj.momentFuncCorrHandle = @(stateMean, stateCov, stateCovSqrt, ...
                                         stateNoiseCrossCov) ...
                                       obj.momentFuncCorrAddNoiseMeasModel(measModel, dimMeas, ...
                                                                           addNoiseMean, addNoiseCov, ...
                                                                           stateMean, stateCov, stateCovSqrt, ...
                                                                           stateNoiseCrossCov);
        end
        
        function setupMixedNoiseMeasModel(obj, measModel, dimMeas)
            [noiseMean, noiseCov, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            [addNoiseMean, addNoiseCov]  = measModel.additiveNoise.getMeanAndCov();
            dimNoise    = size(noiseMean, 1);
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.momentFuncHandle = @(stateMean, stateCov, stateCovSqrt) ...
                                   obj.momentFuncMixedNoiseMeasModel(measModel, dimMeas, dimNoise, ...
                                                                     noiseMean, noiseCov, noiseCovSqrt, ...
                                                                     addNoiseMean, addNoiseCov, ...
                                                                     stateMean, stateCov, stateCovSqrt);
            
            obj.momentFuncCorrHandle = @(stateMean, stateCov, stateCovSqrt, ...
                                         stateNoiseCrossCov) ...
                                       obj.momentFuncCorrMixedNoiseMeasModel(measModel, dimMeas, dimNoise, ...
                                                                             noiseMean, noiseCov, noiseCovSqrt, ...
                                                                             addNoiseMean, addNoiseCov, ...
                                                                             stateMean, stateCov, stateCovSqrt, ...
                                                                             stateNoiseCrossCov);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoiseCrossCov] = getMeasMoments(obj, priorStateMean, priorStateCov, priorStateCovSqrt)
            [measMean, measCov, ...
             stateMeasCrossCov, ...
             measNoiseCrossCov] = obj.momentFuncHandle(priorStateMean, priorStateCov, priorStateCovSqrt);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoiseCrossCov] = getMeasMomentsCorr(obj, updatedStateMean, updatedStateCov, updatedStateCovSqrt, ...
                                                          updatedStateNoiseCrossCov)
            [measMean, measCov, ...
             stateMeasCrossCov, ...
             measNoiseCrossCov] = obj.momentFuncCorrHandle(updatedStateMean, updatedStateCov, updatedStateCovSqrt, ...
                                                           updatedStateNoiseCrossCov);
        end
    end
    
    methods (Access = 'private')
        % Measurement update moment computation with uncorrelated system state and measurement noise
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoiseCrossCov] = momentFuncMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                           noiseMean, noiseCov, noiseCovSqrt, ...
                                                           stateMean, stateCov, stateCovSqrt)
            % No correlations between system state and measurement noise:
            % E[(x - E[x])*(v - E[v])'] = 0
            
            [measMean, stateJacobian, ...
             noiseJacobian] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                    noiseMean, stateMean);
            
            % Measurement covariance matrix:
            Px = stateJacobian * stateCovSqrt;
            Pv = noiseJacobian * noiseCovSqrt;
            
            measCov = Px * Px' + Pv * Pv';
            
            % State--measurement cross-covariance matrix:
            stateMeasCrossCov = stateCov * stateJacobian';
            
            % Measurement--noise cross-covariance matrix:
            measNoiseCrossCov = noiseJacobian * noiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoiseCrossCov] = momentFuncAddNoiseMeasModel(obj, measModel, dimMeas, ...
                                                                   addNoiseMean, addNoiseCov, ...
                                                                   stateMean, stateCov, stateCovSqrt)
            % No correlations between system state and measurement noise:
            % E[(x - E[x])*(v - E[v])'] = 0
            
            [h, stateJacobian] = obj.evaluateAddNoiseMeasModel(measModel, dimMeas, stateMean);
            
            % Measurement mean:
            measMean = h + addNoiseMean;
            
            % Measurement covariance matrix:
            Px = stateJacobian * stateCovSqrt;
            
            measCov = Px * Px' + addNoiseCov;
            
            % State--measurement cross-covariance matrix:
            stateMeasCrossCov = stateCov * stateJacobian';
            
            % Measurement--noise cross-covariance matrix:
            measNoiseCrossCov = addNoiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoisesCrossCov] = momentFuncMixedNoiseMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                                      noiseMean, noiseCov, noiseCovSqrt, ...
                                                                      addNoiseMean, addNoiseCov, ...
                                                                      stateMean, stateCov, stateCovSqrt)
            % No correlations between system state and measurement noise:
            % E[(x - E[x])*(v - E[v])'] = 0
            % E[(x - E[x])*(r - E[r])'] = 0
            
            [h, stateJacobian, ...
             noiseJacobian] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                    noiseMean, stateMean);
            
            % Measurement mean:
            measMean = h + addNoiseMean;
            
            % Measurement covariance matrix:
            Px = stateJacobian * stateCovSqrt;
            Pv = noiseJacobian * noiseCovSqrt;
            
            measCov = Px * Px' + Pv * Pv' + addNoiseCov;
            
            % State--measurement cross-covariance matrix:
            stateMeasCrossCov = stateCov * stateJacobian';
            
            % Measurement--noise cross-covariance matrix:
            measNoisesCrossCov = [noiseJacobian * noiseCov addNoiseCov];
        end
        
        % Measurement update moment computation with correlated system state and measurement noise
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoiseCrossCov] = momentFuncCorrMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                               noiseMean, noiseCov, noiseCovSqrt, ...
                                                               stateMean, stateCov, stateCovSqrt, ...
                                                               stateNoiseCrossCov)
            % Correlations between system state and measurement noise:
            % E[(x - E[x])*(v - E[v])'] != 0
            
            [measMean, stateJacobian, ...
             noiseJacobian] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                    noiseMean, stateMean);
            
            % Measurement covariance matrix:
            Px = stateJacobian * stateCovSqrt;
            Pv = noiseJacobian * noiseCovSqrt;
            Av = stateJacobian * stateNoiseCrossCov * noiseJacobian';
            
            measCov = Px * Px' + Pv * Pv' + (Av + Av');
            
            % State--measurement cross-covariance matrix:
            stateMeasCrossCov = stateCov * stateJacobian' + stateNoiseCrossCov * noiseJacobian';
            
            % Measurement--noise cross-covariance matrix:
            measNoiseCrossCov = stateJacobian * stateNoiseCrossCov + noiseJacobian * noiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoiseCrossCov] = momentFuncCorrAddNoiseMeasModel(obj, measModel, dimMeas, ...
                                                                       addNoiseMean, addNoiseCov, ...
                                                                       stateMean, stateCov, stateCovSqrt, ...
                                                                       stateAddNoiseCrossCov)
            % Correlations between system state and measurement noise:
            % E[(x - E[x])*(v - E[v])'] != 0
            
            [h, stateJacobian] = obj.evaluateAddNoiseMeasModel(measModel, dimMeas, stateMean);
            
            % Measurement mean:
            measMean = h + addNoiseMean;
            
            % Measurement covariance matrix:
            Px = stateJacobian * stateCovSqrt;
            Av = stateJacobian * stateAddNoiseCrossCov;
            
            measCov = Px * Px' + (Av + Av') + addNoiseCov;
            
            % State--measurement cross-covariance matrix:
            stateMeasCrossCov = stateCov * stateJacobian' + stateAddNoiseCrossCov;
            
            % Measurement--noise cross-covariance matrix:
            measNoiseCrossCov = Av + addNoiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov, ...
                  measNoisesCrossCov] = momentFuncCorrMixedNoiseMeasModel(obj, measModel, dimMeas, dimNoise, ...
                                                                          noiseMean, noiseCov, noiseCovSqrt, ...
                                                                          addNoiseMean, addNoiseCov, ...
                                                                          stateMean, stateCov, stateCovSqrt, ...
                                                                          stateNoisesCrossCov)
            % Correlations between system state and measurement noise:
            % E[(x - E[x])*(v - E[v])'] != 0
            % E[(x - E[x])*(r - E[r])'] != 0
            
            stateNoiseCrossCov    = stateNoisesCrossCov(:, 1:dimNoise);
            stateAddNoiseCrossCov = stateNoisesCrossCov(:, dimNoise+1:end);
            
            [h, stateJacobian, ...
             noiseJacobian] = obj.evaluateMeasModel(measModel, dimMeas, dimNoise, ...
                                                    noiseMean, stateMean);
            
            % Measurement mean:
            measMean = h + addNoiseMean;
            
            % Measurement covariance matrix:
            Px = stateJacobian * stateCovSqrt;
            Pv = noiseJacobian * noiseCovSqrt;
            Av = stateJacobian * stateNoiseCrossCov * noiseJacobian';
            Ar = stateJacobian * stateAddNoiseCrossCov;
            
            measCov = Px * Px' + Pv * Pv' + (Av + Av') + (Ar + Ar') + addNoiseCov;
            
            % State--measurement cross-covariance matrix:
            stateMeasCrossCov = stateCov * stateJacobian' + ...
                                stateNoiseCrossCov * noiseJacobian' + ...
                                stateAddNoiseCrossCov;
            
            % Measurement--noise cross-covariance matrix:
            measNoiseCrossCov    = stateJacobian * stateNoiseCrossCov + noiseJacobian * noiseCov;
            measAddNoiseCrossCov = Ar + addNoiseCov;
            
            measNoisesCrossCov = [measNoiseCrossCov measAddNoiseCrossCov];
        end
    end
    
    properties (Access = 'private')
        % Function handle to the currently used moment computation method
        % for uncorrelated system state and measurement noise.
        momentFuncHandle;
        
        % Function handle to the currently used moment computation method
        % for correlated system state and measurement noise.
        momentFuncCorrHandle;
    end
end
