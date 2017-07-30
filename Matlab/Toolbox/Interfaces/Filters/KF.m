
classdef KF < GaussianFilter
    % Abstract base class for Kalman filters (KFs).
    %
    % This type of filter uses the Kalman formulas to perform a measurement update (i.e., a
    % linear filter). This means in case of a nonlinear measurement model/non-Gaussian
    % measurement noise that the posterior state estimate will always be an approximation.
    %
    % KF Methods:
    %   KF                         - Class constructor.
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
    %   Pawe Stano, Zsófia Lendek, Jelmer Braaksma, Robert Babuska, Cees de Keizer, and Arnold J. den Dekker,
    %   Parametric Bayesian Filters for Nonlinear Stochastic Dynamical Systems: A Survey,
    %   IEEE Transactions on Cybernetics, Vol. 43, No. 6, Dec 2013, pp. 1607-1624.
    %
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
        function obj = KF(name)
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
            %   << obj (KF)
            %      A new KF instance.
            
            % Call superclass constructor
            obj = obj@GaussianFilter(name);
            
            % By default, no measurement validation
            obj.measValidationThreshold = 1;
            
            % By default, only one iteration is performed during a
            % measurement update, i.e., the usual non-iterated update.
            obj.maxNumIterations = 1;
            
            obj.lastNumIterations = 0;
        end
        
        function setMaxNumIterations(obj, maxNumIterations)
            % Set the maximum number of iterations that will be performed during a measurement update.
            %
            % By default, only one iteration is performed during a measurement update,
            % i.e., the usual non-iterated update.
            %
            % Parameters:
            %   >> maxNumIterations (Positive scalar)
            %      The new maximum number of iterations that will be performed during a measurement update.
            
            if ~Checks.isPosScalar(maxNumIterations)
                obj.error('InvalidMeasurementValidationThreshold', ...
                          'maxNumIterations must be a positive scalar.');
            end
            
            maxNumIterations = ceil(maxNumIterations);
            
            obj.maxNumIterations = maxNumIterations;
        end
        
        function maxNumIterations = getMaxNumIterations(obj)
            % Get the current maximum number of iterations that will be performed during a measurement update.
            %
            % Returns:
            %   << maxNumIterations (Positive scalar)
            %      The current maximum number of iterations that will be performed during a measurement update.
            
            maxNumIterations = obj.maxNumIterations;
        end
        
        function setMeasValidationThreshold(obj, threshold)
            % Set a threshold to perform a measurement validation (measurement acceptance/rejection).
            %
            % A value of zero means that only measurements, which exactly match the expected
            % measurement mean, will be accepted. In contrast, a value of one means that all
            % measurements will be accepted (i.e., no validation will be performed at all).
            %
            % By default, the validation threshold is set to 1 (i.e., no validation).
            %
            % The validation is based on the Normalized Innovation Squared (NIS) as defined in
            %   Yaakov Bar-Shalom, X. Rong Li, and Thiagalingam Kirubarajan,
            %   Estimation with Applications to Tracking and Navigation,
            %   Wiley-Interscience, 2001, page 236
            %
            % Parameters:
            %   >> threshold (Scalar in [0, 1])
            %      The new measurement validation threshold.
            
            if ~Checks.isScalarIn(threshold, 0, 1)
                obj.error('InvalidMeasurementValidationThreshold', ...
                          'threshold must be a scalar in [0, 1].');
            end
            
            obj.measValidationThreshold = threshold;
        end
        
        function threshold = getMeasValidationThreshold(obj)
            % Get the current measurement validation threshold.
            %
            % Returns:
            %   << threshold (Scalar in [0, 1])
            %      The current measurement validation threshold.
            
            threshold = obj.measValidationThreshold;
        end
        
        function [measurement, ...
                  measMean, ...
                  measCov, ...
                  stateMeasCrossCov, ...
                  numIterations] = getLastUpdateData(obj)
            % Get information from the last performed measurement update.
            %
            % Returns:
            %   << measurement (Column vector)
            %      The measurement vector used by the last measurement update.
            %
            %   << measMean (Column vector)
            %      The measurement mean used by the last measurement update.
            %
            %   << measCov (Positive definite matrix)
            %      The measurement covariance used by the last measurement update.
            %
            %   << stateMeasCrossCov (Matrix)
            %      The state measurement cross-covariance used by the last measurement update.
            %
            %   << numIterations (Scalar)
            %      The number of iterations performed during the last measurement update.
            
            measurement       = obj.lastMeasurement;
            measMean          = obj.lastMeasMean;
            measCov           = obj.lastMeasCov;
            stateMeasCrossCov = obj.lastStateMeasCrossCov;
            numIterations     = obj.lastNumIterations;
        end
    end
    
    methods (Abstract, Access = 'protected')
        momentFunc = getMomentFuncArbitraryNoise(obj, measModel, measurement);
        
        momentFunc = getMomentFuncAdditiveNoise(obj, measModel, measurement);
        
        momentFunc = getMomentFuncMixedNoise(obj, measModel, measurement);
    end
    
    methods (Access = 'protected')
        function [updatedMean, ...
                  updatedCov] = performUpdateObservable(obj, measModel, measurement, ...
                                                        priorMean, priorCov, priorCovSqrt)
            obj.checkMeasurementVector(measurement);
            
            if Checks.isClass(measModel, 'LinearMeasurementModel')
                momentFunc = obj.getMomentFuncLinear(measModel);
            elseif Checks.isClass(measModel, 'MeasurementModel')
                momentFunc = obj.getMomentFuncArbitraryNoise(measModel, measurement);
            elseif Checks.isClass(measModel, 'AdditiveNoiseMeasurementModel')
                momentFunc = obj.getMomentFuncAdditiveNoise(measModel, measurement);
            elseif Checks.isClass(measModel, 'MixedNoiseMeasurementModel')
                momentFunc = obj.getMomentFuncMixedNoise(measModel, measurement);
            else
                obj.errorMeasModel('LinearMeasurementModel', ...
                                   'MeasurementModel', ...
                                   'AdditiveNoiseMeausrementModel', ...
                                   'MixedNoiseMeasurementModel');
            end
            
            [updatedMean, ...
             updatedCov] = obj.kalmanUpdate(measurement, momentFunc, ...
                                            priorMean, priorCov, priorCovSqrt);
        end
        
        function [updatedMean, ...
                  updatedCov] = kalmanUpdate(obj, measurement, momentFunc, ...
                                             priorMean, priorCov, priorCovSqrt)
            iterNum = 1;
            
            [measMean, measCov, ...
             stateMeasCrossCov] = momentFunc(priorMean, priorCov, priorCovSqrt, ...
                                             iterNum, priorMean, priorCov, priorCovSqrt);
            
            while iterNum < obj.maxNumIterations
                try
                    [iterMean, iterCov] = Utils.kalmanUpdate(priorMean, priorCov, measurement, ...
                                                             measMean, measCov, stateMeasCrossCov);
                catch ex
                    obj.ignoreMeas(ex.message);
                end
                
                % Check intermediate state covariance is valid
                [isPosDef, iterCovSqrt] = Checks.isCov(iterCov);
                
                if ~isPosDef
                    obj.ignoreMeas('Intermediate state covariance matrix is not positive definite.');
                end
                
                iterNum = iterNum + 1;
                
                [measMean, measCov, ...
                 stateMeasCrossCov] = momentFunc(priorMean, priorCov, priorCovSqrt, ...
                                                 iterNum, iterMean, iterCov, iterCovSqrt);
            end
            
            % Save Kalman update information
            obj.lastMeasurement       = measurement;
            obj.lastMeasMean          = measMean;
            obj.lastMeasCov           = measCov;
            obj.lastStateMeasCrossCov = stateMeasCrossCov;
            obj.lastNumIterations     = iterNum;
            
            % Perform gating if enabled
            if obj.measValidationThreshold ~= 1
                obj.measurementGating(measurement, measMean, measCov);
            end
            
            try
                [updatedMean, ...
                 updatedCov] = Utils.kalmanUpdate(priorMean, priorCov, measurement, ...
                                                  measMean, measCov, stateMeasCrossCov);
            catch ex
                obj.ignoreMeas(ex.message);
            end
        end
        
        function momentFunc = getMomentFuncLinear(obj, measModel)
            momentFunc = @(priorMean, priorCov, priorCovSqrt, iterNum, iterMean, iterCov, iterCovSqrt) ...
                         obj.momentFuncLinear(priorMean, priorCov, priorCovSqrt, ...
                                              iterNum, iterMean, iterCov, iterCovSqrt, measModel);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentFuncLinear(~, priorMean, priorCov, priorCovSqrt, ...
                                                        iterNum, iterMean, iterCov, iterCovSqrt, measModel)
            % Compute measurement moments
            [measMean, measCov, ...
             stateMeasCrossCov] = measModel.analyticMoments(iterMean, iterCov, iterCovSqrt);
            
            if iterNum > 1
                [measMean, measCov, ...
                 stateMeasCrossCov] = KF.momentCorrection(priorMean, priorCov, priorCovSqrt, ...
                                                          iterMean, iterCov, iterCovSqrt, ...
                                                          measMean, measCov, stateMeasCrossCov);
            end
        end
    end
    
    methods (Static, Access = 'protected')
        function [measMean, measCov, ...
                  stateMeasCrossCov] = momentCorrection(priorMean, priorCov, priorCovSqrt, ...
                                                        iterMean, iterCov, iterCovSqrt, ...
                                                        measMean, measCov, stateMeasCrossCov)
            A = stateMeasCrossCov' / iterCov;
            B = A * priorCovSqrt;
            C = A * iterCovSqrt;
            
            measMean          = measMean + A * (priorMean - iterMean);
            measCov           = measCov + B * B' - C * C';
            stateMeasCrossCov = priorCov * A';
        end
    end
    
    methods (Access = 'private')
        function measurementGating(obj, measurement, measMean, measCov)
            % Compute Mahalanobis distance
            dimMeas = size(measurement, 1);
            
            t = measurement - measMean;
            
            value = t' * (measCov \ t);
            
            % Normalize distance
            normalizedValue = chi2cdf(value, dimMeas);
            
            % Check for threshold exceedance
            if normalizedValue > obj.measValidationThreshold
                % Discard measurement
                obj.ignoreMeas('Measurement exceeds validation threshold (value: %f).', ...
                               normalizedValue);
            end
        end
    end
    
    properties (Access = 'private')
        % The maximum number of iterations that will be performed during a measurement update.
        maxNumIterations;
        
        % The measurement validation threshold.
        measValidationThreshold;
        
        % The measurement vector used by the last measurement update.
        lastMeasurement;
        
        % The measurement mean used by the last filter measurement update.
        lastMeasMean;
        
        % The measurement covariance used by the last measurement update.
        lastMeasCov;
        
        % The state measurement cross-covariance used by the last measurement update.
        lastStateMeasCrossCov;
        
        % The number of iterations performed during the last measurement update.
        lastNumIterations;
    end
end
