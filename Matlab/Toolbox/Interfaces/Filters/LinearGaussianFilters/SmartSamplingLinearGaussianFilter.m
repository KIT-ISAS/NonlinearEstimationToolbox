
classdef SmartSamplingLinearGaussianFilter < SampleBasedLinearGaussianFilter
    % Abstract base class for linear Gaussian filters that are based on the
    % point-symmetric LCD-based Gaussian sampling technique.
    %
    % SmartSamplingLinearGaussianFilter Methods:
    %   SmartSamplingLinearGaussianFilter - Class constructor.
    %   copy                              - Copy a Filter instance.
    %   copyWithName                      - Copy a Filter instance and give the copy a new name/description.
    %   getName                           - Get the filter name/description.
    %   setColor                          - Set the filter color/plotting properties.
    %   getColor                          - Get the filter color/plotting properties.
    %   setState                          - Set the system state.
    %   getState                          - Get the system state.
    %   getStateDim                       - Get the dimension of the system state.
    %   getStateMeanAndCov                - Get mean and covariance matrix of the system state.
    %   predict                           - Perform a state prediction.
    %   update                            - Perform a measurement update.
    %   step                              - Perform a combined state prediction and measurement update.
    %   setStateDecompDim                 - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim                 - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing       - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing       - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing           - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing           - Get the post-processing method for the measurement update.
    %   setMeasGatingThreshold            - Set the measurement gating threshold.
    %   getMeasGatingThreshold            - Get the measurement gating threshold.
    %   setNumSamples                     - Set absolute numbers of samples used for state prediction and measurement update.
    %   setNumSamplesByFactors            - Set linear factors to determine the number of samples used for state prediction and measurement update.
    %   getNumSamplesConfigPrediction     - Get the number of samples configuration used for the state prediction.
    %   getNumSamplesConfigUpdate         - Get the number of samples configuration used for the measurement update.
    
    % Literature:
    %   Jannik Steinbring, Martin Pander, and Uwe D. Hanebeck,
    %   The Smart Sampling Kalman Filter with Symmetric Samples
    %   Journal of Advances in Information Fusion, vol. 11, no. 1, pp. 71-90, Jun. 2016.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   LRKF Revisited: The Smart Sampling Kalman Filter (S²KF),
    %   Journal of Advances in Information Fusion, vol. 9, no. 2, pp. 106-123, Dec. 2014.
    
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
        function obj = SmartSamplingLinearGaussianFilter(name)
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
            %   << obj (SmartSamplingLinearGaussianFilter)
            %      A new SmartSamplingLinearGaussianFilter instance.
            
            samplingPred = GaussianSamplingLCD();
            samplingUp   = GaussianSamplingLCD();
            
            % Call superclass constructor
            obj = obj@SampleBasedLinearGaussianFilter(name, samplingPred, samplingUp);
            
            % The point-symmetric LCD-based sampling is used.
            obj.samplingPrediction.setSymmetricMode(true);
            obj.samplingUpdate.setSymmetricMode(true);
            
            % By default, determine the number of samples for state prediction
            % and measurement update by using a linear factor of 10.
            obj.samplingPrediction.setNumSamplesByFactor(10);
            obj.samplingUpdate.setNumSamplesByFactor(10);
        end
        
        function setNumSamples(obj, numSamplesPrediction, numSamplesUpdate)
            % Set absolute numbers of samples used for state prediction and measurement update.
            %
            % This also overwrites a possible previous setting, where the number of
            % samples are determined by a linear factor (see setNumSamplesByFactor()).
            %
            % By default, a linear factor 10 is used for both state prediction
            % and measurement update.
            %
            % Parameters:
            %   >> numSamplesPrediction (Positive scalar or empty matrix)
            %      The new absolute number of samples used for the state prediction.
            %      Pass an empty matrix to keep the current configuration for the
            %      state prediction.
            %
            %   >> numSamplesUpdate (Positive scalar or empty matrix)
            %      The new absolute number of samples used for the measurement update.
            %      Pass an empty matrix to keep the current configuration for the
            %      measurement update. If nothing is passed, the absolute number of
            %      samples specified for the state prediction is also used for the
            %      measurement update.
            
            if ~isempty(numSamplesPrediction)
                obj.samplingPrediction.setNumSamples(numSamplesPrediction);
            end
            
            if nargin == 3
                if ~isempty(numSamplesUpdate)
                    obj.samplingUpdate.setNumSamples(numSamplesUpdate);
                end
            elseif ~isempty(numSamplesPrediction)
                obj.samplingUpdate.setNumSamples(numSamplesPrediction);
            end
        end
        
        function setNumSamplesByFactors(obj, factorPrediction, factorUpdate)
            % Set linear factors to determine the number of samples used for state prediction and measurement update.
            %
            % The actual number of samples will be computed according to
            %
            %    Number of samples = factor * dimension + 1 - mod(factor * dimension, 2)
            %
            % i.e., always an odd number of samples is used.
            %
            % This also overwrites a possible previous setting, where the number of
            % samples are determined in an absolute way (see setNumSamples()).
            %
            % By default, a linear factor of 10 is used for both state prediction
            % and measurement update.
            %
            % Parameters:
            %   >> factorPrediction (Positive scalar or empty matrix)
            %      The new linear factor to determine the number of samples used for
            %      the state prediction. Pass an empty matrix to keep the current
            %      configuration for the state prediction.
            %
            %   >> factorUpdate (Positive scalar or empty matrix)
            %      The new linear factor to determine the number of samples used for
            %      the measurement update. Pass an empty matrix to keep the current
            %      configuration for the measurement update. If nothing is passed, the
            %      linear factor specified for the state prediction is also used for
            %      the measurement update.
            
            if ~isempty(factorPrediction)
                obj.samplingPrediction.setNumSamplesByFactor(factorPrediction);
            end
            
            if nargin == 3
                if ~isempty(factorUpdate)
                    obj.samplingUpdate.setNumSamplesByFactor(factorUpdate);
                end
            elseif ~isempty(factorPrediction)
                obj.samplingUpdate.setNumSamplesByFactor(factorPrediction);
            end
        end
        
        function [numSamplesAbs, ...
                  numSamplesFactor] = getNumSamplesConfigPrediction(obj)
            % Get the number of samples configuration used for the state prediction.
            %
            % Returns:
            %   << numSamplesAbs (Positive scalar or empty matrix)
            %      Equals the absolute number of samples if set.
            %      Otherwise, an empty matrix.
            %
            %   << numSamplesFactor (Positive scalar or empty matrix)
            %      Equals the sample factor if set.
            %      Otherwise, an empty matrix.
            
            [numSamplesAbs, ...
             numSamplesFactor] = obj.samplingPrediction.getNumSamplesConfig();
        end
        
        function [numSamplesAbs, ...
                  numSamplesFactor] = getNumSamplesConfigUpdate(obj)
            % Get the number of samples configuration used for the measurement update.
            %
            % Returns:
            %   << numSamplesAbs (Positive scalar or empty matrix)
            %      Equals the absolute number of samples if set.
            %      Otherwise, an empty matrix.
            %
            %   << numSamplesFactor (Positive scalar or empty matrix)
            %      Equals the sample factor if set.
            %      Otherwise, an empty matrix.
            
            [numSamplesAbs, ...
             numSamplesFactor] = obj.samplingUpdate.getNumSamplesConfig();
        end
    end
    
    methods (Access = 'protected')
        % We overwrite the moment computations for the state prediction and
        % the measurement update in order to exploit the fact that all samples
        % of the LCD-based Gaussian sampling are equally weighted. This reduces
        % runtime and has numerical benefits.
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictSysModel(obj, sysModel)
            [noiseMean, ~, noiseCovSqrt] = sysModel.noise.getMeanAndCov();
            
            % Generate state and noise samples
            % Keep in mind that we know that all samples are equally weighted
            [stateSamples, ...
             noiseSamples, ...
             ~, ...
             numSamples] = Utils.getStateNoiseSamples(obj.samplingPrediction, ...
                                                      obj.stateMean, obj.stateCovSqrt, ...
                                                      noiseMean, noiseCovSqrt);
            
            % Propagate samples through system equation
            predictedStates = sysModel.systemEquation(stateSamples, noiseSamples);
            
            % Check predicted state samples
            obj.checkPredictedStateSamples(predictedStates, numSamples);
            
            % Compute predicted state mean and covariance
            [predictedStateMean, ...
             predictedStateCov] = Utils.getMeanAndCov(predictedStates);
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictAddNoiseSysModel(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCov();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Generate state samples
            % Keep in mind that we know that all samples are equally weighted
            [stateSamples, ...
             ~, ...
             numSamples] = Utils.getStateSamples(obj.samplingPrediction, ...
                                                 obj.stateMean, obj.stateCovSqrt);
            
            % Propagate samples through deterministic system equation
            predictedStates = sysModel.systemEquation(stateSamples);
            
            % Check predicted state samples
            obj.checkPredictedStateSamples(predictedStates, numSamples);
            
            [mean, cov] = Utils.getMeanAndCov(predictedStates);
            
            % Compute predicted state mean
            predictedStateMean = mean + noiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + noiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMeasModel(obj, stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                           noiseMean, noiseCovSqrt)
            % Generate state and noise samples
            % Keep in mind that we know that all samples are equally weighted
            [stateSamples, ...
             noiseSamples, ...
             ~, ...
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
                                                              measSamples);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncAddNoiseMeasModel(obj, stateMean, stateCovSqrt, measModel, dimMeas, ...
                                                                   addNoiseMean, addNoiseCov)
            % Generate state samples
            % Keep in mind that we know that all samples are equally weighted
            [stateSamples, ...
             ~, ...
             numSamples] = Utils.getStateSamples(obj.samplingUpdate, ...
                                                 stateMean, stateCovSqrt);
            
            % Propagate samples through deterministic measurement equation
            measSamples = measModel.measurementEquation(stateSamples);
            
            % Check computed measurements
            obj.checkComputedMeasurements(measSamples, dimMeas, numSamples);
            
            % Compute moments
            [mean, cov, ...
             stateMeasCrossCov] = Utils.getMeanCovAndCrossCov(stateMean, stateSamples, ...
                                                              measSamples);
            
            measMean = mean + addNoiseMean;
            
            measCov = cov + addNoiseCov;
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncMixedNoiseMeasModel(obj, stateMean,stateCovSqrt, measModel, dimMeas, ...
                                                                     noiseMean, noiseCovSqrt, addNoiseMean, addNoiseCov)
            % Generate state and noise samples
            % Keep in mind that we know that all samples are equally weighted
            [stateSamples, ...
             noiseSamples, ...
             ~, ...
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
                                                              measSamples);
            
            measMean = mean + addNoiseMean;
            
            measCov = cov + addNoiseCov;
        end
    end
end
