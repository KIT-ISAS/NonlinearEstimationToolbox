
classdef PGF < GaussianFilter
    % The progressive Gaussian filter (PGF).
    %
    % PGF Methods:
    %   PGF                           - Class constructor.
    %   copy                          - Copy a Filter instance.
    %   copyWithName                  - Copy a Filter instance and give the copy a new name/description.
    %   getName                       - Get the filter name/description.
    %   setColor                      - Set the filter color/plotting properties.
    %   getColor                      - Get the filter color/plotting properties.
    %   setState                      - Set the system state.
    %   getState                      - Get the system state.
    %   getStateDim                   - Get the dimension of the system state.
    %   getStateMeanAndCov            - Get mean and covariance matrix of the system state.
    %   predict                       - Perform a state prediction.
    %   update                        - Perform a measurement update.
    %   step                          - Perform a combined state prediction and measurement update.
    %   setStateDecompDim             - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim             - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing   - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing   - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing       - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing       - Get the post-processing method for the measurement update.
    %   setNumSamples                 - Set absolute numbers of samples used for state prediction and measurement update.
    %   setNumSamplesByFactors        - Set linear factors to determine the number of samples used for state prediction and measurement update.
    %   getNumSamplesConfigPrediction - Get the number of samples configuration used for the state prediction.
    %   getNumSamplesConfigUpdate     - Get the number of samples configuration used for the measurement update.
    %   setMaxNumProgSteps            - Set the maximum number of allowed progression steps.
    %   getMaxNumProgSteps            - Get the maximum number of allowed progression steps.
    %   getNumProgSteps               - Get the number of progression steps required by the last measurement update.
    
    % Literature:
    %   Jannik Steinbring, Antonio Zea, and Uwe D. Hanebeck,
    %   Semi-Analytic Progressive Gaussian Filtering,
    %   Proceedings of the 2016 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI 2016), Baden-Baden, Germany, Sep 2016.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   GPU-Accelerated Progressive Gaussian Filtering with Applications to Extended Object Tracking,
    %   Proceedings of the 18th International Conference on Information Fusion (Fusion 2015), Washington D. C., USA, Jul. 2015.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   Progressive Gaussian Filtering Using Explicit Likelihoods,
    %   Proceedings of the 17th International Conference on Information Fusion (Fusion 2014), Salamanca, Spain, Jul. 2014.
    
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
        function obj = PGF(name)
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
            %      Default name: 'PGF'.
            %
            % Returns:
            %   << obj (PGF)
            %      A new PGF instance.
            
            if nargin < 1
                name = 'PGF';
            end
            
            % Call superclass constructor
            obj = obj@GaussianFilter(name);
            
            obj.samplingPrediction = GaussianSamplingLCD();
            obj.samplingUpdate     = GaussianSamplingLCD();
            
            % The point-symmetric LCD-based sampling is used.
            obj.samplingPrediction.setSymmetricMode(true);
            obj.samplingUpdate.setSymmetricMode(true);
            
            % By default, determine the number of samples for state prediction
            % and measurement update by using a linear factor of 10.
            obj.samplingPrediction.setNumSamplesByFactor(10);
            obj.samplingUpdate.setNumSamplesByFactor(10);
            
            obj.maxNumProgSteps = 0;
            obj.numProgSteps    = 0;
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
        
        function setMaxNumProgSteps(obj, maxNumProgSteps)
            % Set the maximum number of allowed progression steps.
            %
            % If a measurement update exceeds the maximum number of allowed
            % progression steps, the progression will be stopped and the
            % current intermediate Gaussian distribution will be set as the
            % posterior state estimate. This can be interpreted as not the
            % entire information of the given measurements were processed.
            %
            % A maximum number is useful when certain runtime conditions
            % have to be met, e.g., an update has to be performed within a
            % specified time.
            %
            % A value of zero means no maximum, and hence, the progression
            % will not be interrupted.
            %
            % By default, no maximum is set.
            %
            % Parameters:
            %   >> maxNumProgSteps (Scalar)
            %      The new maximum number of allowed progression steps.
            
            if ~Checks.isScalar(maxNumProgSteps)
                obj.error('InvalidMaximumNumberOfProgressionSteps', ...
                          'maxNumProgSteps must be a scalar.');
            end
            
            obj.maxNumProgSteps = ceil(maxNumProgSteps);
        end
        
        function maxNumProgSteps = getMaxNumProgSteps(obj)
            % Get the maximum number of allowed progression steps.
            %
            % Returns:
            %   << maxNumProgSteps (Scalar)
            %      The maximum number of allowed progression steps.
            
            maxNumProgSteps = obj.maxNumProgSteps;
        end
        
        function numProgSteps = getNumProgSteps(obj)
            % Get the number of progression steps required by the last measurement update.
            %
            % Returns:
            %   << numProgSteps (Non-negative scalar)
            %      The number of progression steps required by the last measurement update.
            
            numProgSteps = obj.numProgSteps;
        end
    end
    
    methods (Access = 'protected')
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
        
        function [updatedMean, ...
                  updatedCov] = performUpdateObservable(obj, measModel, measurement, ...
                                                        priorMean, ~, priorCovSqrt)
            if Checks.isClass(measModel, 'Likelihood')
                [updatedMean, ...
                 updatedCov] = obj.updateLikelihood(measModel, measurement, ...
                                                    priorMean, priorCovSqrt);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
        
        function [updatedMean, ...
                  updatedCov] = updateLikelihood(obj, measModel, measurement, ...
                                                 priorMean, priorCovSqrt)
            % Initialize progression
            dimState         = size(priorMean, 1);
            updatedMean      = priorMean;
            updatedCovSqrt   = priorCovSqrt;
            gamma            = 0;
            obj.numProgSteps = 0;
            
            % Get standard normal approximation
            [stdNormalSamples, ~, numSamples] = obj.samplingUpdate.getStdNormalSamples(dimState);
            
            % Set forced sample weight ratio
            logForcedRatio = -log(numSamples); % = log(1 / numSamples)
            
            % Start progression
            while gamma < 1 && (obj.maxNumProgSteps == 0 || obj.numProgSteps < obj.maxNumProgSteps)
                % Increment step counter
                obj.numProgSteps = obj.numProgSteps + 1;
                
                % Sample intermediate Gaussian approximation
                samples = updatedCovSqrt * stdNormalSamples;
                samples = bsxfun(@plus, samples, updatedMean);
                
                % Evaluate log likelihood
                logValues = measModel.logLikelihood(samples, measurement);
                
                obj.checkLogLikelihoodEvaluations(logValues, numSamples);
                
                % Compute deltaGamma
                
                % Discard zero likelihood values
                validLogValues = logValues(logValues > -Inf);
                
                if isempty(validLogValues)
                    obj.ignoreMeas('All likelihood values are zero.');
                end
                
                minLogValue = min(validLogValues);
                maxLogValue = max(validLogValues);
                
                if minLogValue == maxLogValue
                    obj.ignoreMeas('Minimum and maximum likelihood values are identical.');
                end
                
                deltaGamma = logForcedRatio / (minLogValue - maxLogValue);
                
                % Clamp deltaGamma if necessary
                if gamma + deltaGamma > 1
                    deltaGamma = 1 - gamma;
                    gamma = 1;
                else
                    gamma = gamma + deltaGamma;
                end
                
                % For numerical stability
                logValues = logValues - maxLogValue;
                
                % Compute intermediate posterior sample weights
                weights = exp(logValues * deltaGamma);
                
                % Normalize weights
                sumWeights = sum(weights);
                
                if sumWeights <= 0
                    obj.ignoreMeas('Sum of computed sample weights is not positive.');
                end
                
                weights = weights / sumWeights;
                
                % Compute new intermediate mean and covariance
                [updatedMean, updatedCov] = Utils.getMeanAndCov(samples, weights);
                
                if gamma ~= 1
                    % Check if intermediate state covariance matrix is valid
                    updatedCovSqrt = obj.checkCovUpdate(updatedCov, 'Intermediate state');
                end
            end
        end
    end
    
    methods (Access = 'protected')
        function cpObj = copyElement(obj)
            cpObj = obj.copyElement@GaussianFilter();
            
            cpObj.samplingPrediction = obj.samplingPrediction.copy();
            cpObj.samplingUpdate     = obj.samplingUpdate.copy();
            cpObj.maxNumProgSteps    = obj.maxNumProgSteps;
            cpObj.numProgSteps       = obj.numProgSteps;
        end
    end
    
    properties (Access = 'private')
        % Gaussian LCD-based sampling technique used for prediction.
        samplingPrediction;
        
        % Gaussian LCD-based sampling technique used for the update.
        samplingUpdate;
        
        % The maximum number of allowed progression steps.
        maxNumProgSteps;
        
        % The number of progression steps required by the last measurement update.
        numProgSteps;
    end
end
