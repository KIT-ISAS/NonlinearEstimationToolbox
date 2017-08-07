
classdef SampleBasedLinearGaussianFilter < LinearGaussianFilter
    % Abstract base class for sample-based linear Gaussian filters.
    %
    % SampleBasedLinearGaussianFilter Methods:
    %   SampleBasedLinearGaussianFilter - Class constructor.
    %   copy                            - Copy a Filter instance.
    %   copyWithName                    - Copy a Filter instance and give the copy a new name/description.
    %   getName                         - Get the filter name/description.
    %   setColor                        - Set the filter color/plotting properties.
    %   getColor                        - Get the filter color/plotting properties.
    %   setState                        - Set the system state.
    %   getState                        - Get the system state.
    %   getStateDim                     - Get the dimension of the system state.
    %   getStateMeanAndCov              - Get mean and covariance matrix of the system state.
    %   predict                         - Perform a state prediction.
    %   update                          - Perform a measurement update.
    %   step                            - Perform a combined state prediction and measurement update.
    %   setStateDecompDim               - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim               - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing     - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing     - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing         - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing         - Get the post-processing method for the measurement update.
    %   setMeasGatingThreshold          - Set the measurement gating threshold.
    %   getMeasGatingThreshold          - Get the measurement gating threshold.
    
    % Literature:
    %   Michael Roth, Gustaf Hendeby, and Fredrik Gustafsson,
    %   Nonlinear Kalman Filters Explained: A Tutorial on Moment Computations and Sigma Point Methods,
    %   Journal of Advances in Information Fusion, vol. 11, no. 1, pp. 47-70, Jun. 2016.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   LRKF Revisited: The Smart Sampling Kalman Filter (S²KF),
    %   Journal of Advances in Information Fusion, vol. 9, no. 2, pp. 106-123, Dec. 2014.
    
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
        function obj = SampleBasedLinearGaussianFilter(name, samplingPrediction, samplingUpdate)
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
            %   >> samplingPrediction (Subclass of GaussianSampling)
            %      The Gaussian sampling method used by the filter to perform a state prediction.
            %
            %   >> samplingUpdate (Subclass of GaussianSampling)
            %      The Gaussian sampling method used by the filter to perform a measurement update.
            %      If no sampling is passed, the sampling for the prediction is also used for the update.
            %
            % Returns:
            %   << obj (SampleBasedLinearGaussianFilter)
            %      A new SampleBasedLinearGaussianFilter instance.
            
            % Call superclass constructor
            obj = obj@LinearGaussianFilter(name);
            
            if ~Checks.isClass(samplingPrediction, 'GaussianSampling')
                obj.error('InvalidGaussianSampling', ...
                          'samplingPrediction must be a subclass of GaussianSampling.');
            end
            
            obj.samplingPrediction = samplingPrediction;
            
            if nargin == 3
                if ~Checks.isClass(samplingUpdate, 'GaussianSampling')
                    obj.error('InvalidGaussianSampling', ...
                              'samplingUpdate must be a subclass of GaussianSampling.');
                end
                
                obj.samplingUpdate = samplingUpdate;
            else
                % Use the same samplings for both prediction and update
                obj.samplingUpdate = samplingPrediction;
            end
        end
    end
    
    methods (Sealed, Access = 'protected')
        function setupMeasModel(obj, measModel, dimMeas)
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            
            obj.measMomentFuncHandle = @(stateMean, stateCovSqrt) ...
                                       obj.momentFuncMeasModel(stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                               noiseMean, noiseCovSqrt);
        end
        
        function setupAddNoiseMeasModel(obj, measModel, dimMeas)
            [addNoiseMean, addNoiseCov] = measModel.noise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.measMomentFuncHandle = @(stateMean, stateCovSqrt) ...
                                       obj.momentFuncAddNoiseMeasModel(stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                                       addNoiseMean, addNoiseCov);
        end
        
        function setupMixedNoiseMeasModel(obj, measModel, dimMeas)
            [noiseMean, ~, noiseCovSqrt] = measModel.noise.getMeanAndCov();
            [addNoiseMean, addNoiseCov]  = measModel.additiveNoise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            obj.measMomentFuncHandle = @(stateMean, stateCovSqrt) ...
                                       obj.momentFuncMixedNoiseMeasModel(stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                                         noiseMean, noiseCovSqrt, addNoiseMean, addNoiseCov);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = measMomentFunc(obj, priorMean, priorCovSqrt)
            % Note that obj.measMomentFuncHandle is set by the setup*() methods,
            % which are called before the actual linear measurement update.
            [measMean, measCov, ...
             stateMeasCrossCov] = obj.measMomentFuncHandle(priorMean, priorCovSqrt);
        end
    end
    
    methods (Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictSysModel(obj, sysModel)
            [noiseMean, ~, noiseCovSqrt] = sysModel.noise.getMeanAndCov();
            
            % Generate state and noise samples
            [stateSamples, ...
             noiseSamples, ...
             weights, ...
             numSamples] = Utils.getStateNoiseSamples(obj.samplingPrediction, ...
                                                      obj.stateMean, obj.stateCovSqrt, ...
                                                      noiseMean, noiseCovSqrt);
            
            % Propagate samples through system equation
            predictedStates = sysModel.systemEquation(stateSamples, noiseSamples);
            
            % Check predicted state samples
            obj.checkPredictedStateSamples(predictedStates, numSamples);
            
            % Compute predicted state mean and covariance
            [predictedStateMean, ...
             predictedStateCov] = Utils.getMeanAndCov(predictedStates, weights);
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictAddNoiseSysModel(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Generate state samples
            [stateSamples, ...
             weights, ...
             numSamples] = Utils.getStateSamples(obj.samplingPrediction, ...
                                                 obj.stateMean, obj.stateCovSqrt);
            
            % Propagate samples through deterministic system equation
            predictedStates = sysModel.systemEquation(stateSamples);
            
            % Check predicted state samples
            obj.checkPredictedStateSamples(predictedStates, numSamples);
            
            [mean, cov] = Utils.getMeanAndCov(predictedStates, weights);
            
            % Compute predicted state mean
            predictedStateMean = mean + noiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + noiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMeasModel(obj, stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                           noiseMean, noiseCovSqrt)
            % Generate state and noise samples
            [stateSamples, ...
             noiseSamples, ...
             weights, ...
             numSamples] = Utils.getStateNoiseSamples(obj.samplingUpdate, ...
                                                      stateMean, stateCovSqrt, ...
                                                      noiseMean, noiseCovSqrt);
            
            % Propagate samples through measurement equation
            measSamples = measModel.measurementEquation(stateSamples, noiseSamples);
            
            % Check computed measurements
            obj.checkComputedMeasurements(measSamples , dimMeas, numSamples);
            
            % Compute moments
            [measMean, measCov, ...
             stateMeasCrossCov] = Utils.getMeanCovAndCrossCov(stateMean, stateSamples, ...
                                                              measSamples, weights);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncAddNoiseMeasModel(obj, stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                                   addNoiseMean, addNoiseCov)
            % Generate state samples
            [stateSamples, ...
             weights, ...
             numSamples] = Utils.getStateSamples(obj.samplingUpdate, ...
                                                 stateMean, stateCovSqrt);
            
            % Propagate samples through deterministic measurement equation
            measSamples = measModel.measurementEquation(stateSamples);
            
            % Check computed measurements
            obj.checkComputedMeasurements(measSamples, dimMeas, numSamples);
            
            % Compute moments
            [mean, cov, ...
             stateMeasCrossCov] = Utils.getMeanCovAndCrossCov(stateMean, stateSamples, ...
                                                              measSamples, weights);
            
            measMean = mean + addNoiseMean;
            
            measCov = cov + addNoiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMixedNoiseMeasModel(obj, stateMean,stateCovSqrt, measModel, dimMeas, ...
                                                                     noiseMean, noiseCovSqrt, addNoiseMean, addNoiseCov)
            % Generate state and noise samples
            [stateSamples, ...
             noiseSamples, ...
             weights, ...
             numSamples] = Utils.getStateNoiseSamples(obj.samplingUpdate, ...
                                                      stateMean, stateCovSqrt, ...
                                                      noiseMean, noiseCovSqrt);
            
            % Propagate samples through measurement equation
            measSamples = measModel.measurementEquation(stateSamples, noiseSamples);
            
            % Check computed measurements
            obj.checkComputedMeasurements(measSamples , dimMeas, numSamples);
            
            % Compute moments
            [mean, cov, ...
             stateMeasCrossCov] = Utils.getMeanCovAndCrossCov(stateMean, stateSamples, ...
                                                              measSamples, weights);
            
            measMean = mean + addNoiseMean;
            
            measCov = cov + addNoiseCov;
        end
        
        function cpObj = copyElement(obj)
            cpObj = obj.copyElement@LinearGaussianFilter();
            
            cpObj.samplingPrediction = obj.samplingPrediction.copy();
            
            if obj.samplingPrediction == obj.samplingUpdate
                % Still use the same samplings for both prediction and update
                cpObj.samplingUpdate = cpObj.samplingPrediction;
            else
                cpObj.samplingUpdate = obj.samplingUpdate.copy();
            end
        end
    end
    
    properties (SetAccess = 'private', GetAccess = 'protected')
        % Gaussian sampling technique used for the state prediction.
        samplingPrediction;
        
        % Gaussian sampling technique used for the measurement update.
        samplingUpdate;
    end
    
    properties (Access = 'private')
        % Function handle to the currently used moment computation method.
        measMomentFuncHandle;
    end
end
