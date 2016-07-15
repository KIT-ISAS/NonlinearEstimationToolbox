
classdef S2KF < LRKF
    % The Smart Sampling Kalman Filter (S²KF)
    %
    % S2KF Methods:
    %   S2KF                           - Class constructor.
    %   getName                        - Get the filter name / description.
    %   setColor                       - Set the filter color / plotting properties.
    %   getColor                       - Get the current filter color / plotting properties.
    %   setState                       - Set the system state.
    %   getState                       - Get the current system state.
    %   getStateDim                    - Get the dimension of the current system state.
    %   predict                        - Perform a time update (prediction step).
    %   update                         - Perform a measurement update (filter step) using the given measurement(s).
    %   step                           - Perform a combined time and measurement update.
    %   getPointEstimate               - Get a point estimate of the current system state.
    %   setStateDecompDim              - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim              - Get the dimension of the unobservable part of the system state.
    %   setMaxNumIterations            - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations            - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold     - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold     - Get the current measurement validation threshold.
    %   getLastUpdateData              - Get information from the last performed measurement update.
    %   setUseAnalyticSystemModel      - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel      - Get the current use of analytic moment calculation during a prediction.
    %   setUseAnalyticMeasurementModel - Enable or disable the use of analytic moment calculation during a filter step.
    %   getUseAnalyticMeasurementModel - Get the current use of analytic moment calculation during a filter step.
    %   setNumSamples                  - Set an absolute number of samples used by the S2KF for prediction and upate.
    %   setNumSamplesByFactor          - Set a linear factor to determine the number of samples used by the S2KF for prediction and upate.
    %   setOnlineMode                  - Select between online and offline sampling.
    %   setSymmetricMode               - Select between symmetric and asymmetric sampling.
    
    % Literature:
    %   Jannik Steinbring, Martin Pander, Uwe D. Hanebeck,
    %   The Smart Sampling Kalman Filter with Symmetric Samples
    %   Journal of Advances in Information Fusion, Vol. 11, No. 1, Jun 2016, pp. 71-90.
    %
    %   Jannik Steinbring and Uwe D. Hanebeck,
    %   LRKF Revisited: The Smart Sampling Kalman Filter (S²KF),
    %   Journal of Advances in Information Fusion, Vol. 9, No. 2, Dec 2014, pp. 106-123.
    
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
        function obj = S2KF(name)
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
            %      Default name: 'S2KF'.
            %
            % Returns:
            %   << obj (S2KF)
            %      A new S2KF instance.
            
            if nargin < 1
                name = 'S2KF';
            end
            
            samplingPred = GaussianSamplingLCD();
            samplingUp   = GaussianSamplingLCD();
            
            obj = obj@LRKF(name, samplingPred, samplingUp);
            
            obj.samplingPrediction = samplingPred;
            obj.samplingUpdate     = samplingUp;
            
            % By default, determine the number of samples for prediction
            % and update by using a factor of 10.
            obj.setNumSamplesByFactor(10);
            
            % By default, disable online mode.
            obj.setOnlineMode(false);
            
            % By default, the symmetric sampling mode is used.
            obj.setSymmetricMode(true);
        end
        
        function setNumSamples(obj, numSamplesPrediction, numSamplesUpdate)
            % Set an absolute number of samples used by the S2KF for prediction and upate.
            %
            % This overwrites a possible previous setting, where the number of samples
            % are determined by a linear factor (see setNumSamplesByFactor()).
            %
            % By default, a linear factor 10 is used for prediction and update.
            %
            % Parameters:
            %    >> numSamplesPrediction (Positive scalar)
            %       The new absolute number of samples used for the prediction.
            %
            %    >> numSamplesUpdate (Positive scalar)
            %       The new absolute number of samples used for the update.
            %       Default: the same number of samples specified for the prediction.
            
            obj.samplingPrediction.setNumSamples(numSamplesPrediction);
            
            if nargin == 3
                obj.samplingUpdate.setNumSamples(numSamplesUpdate);
            else
                obj.samplingUpdate.setNumSamples(numSamplesPrediction);
            end
        end
        
        function setNumSamplesByFactor(obj, factorPrediction, factorUpdate)
            % Set a linear factor to determine the number of samples used by the S2KF for prediction and upate.
            %
            % The actual number of samples will be computed according to
            %
            %    Number of samples = factor * dimension
            %
            % This overwrites a possible previous setting, where the number of samples
            % are determined in an absolute way (see setNumSamples()).
            %
            % By default, a linear factor 10 is used for prediction and update.
            %
            % Parameters:
            %    >> factorPrediction (Positive scalar)
            %       The new linear factor to determine the number of samples for the prediction.
            %
            %    >> factorUpdate (Positive scalar)
            %       The new linear factor to determine the number of samples for the update.
            %       Default: the same factor specified for the prediction.
            
            obj.samplingPrediction.setNumSamplesByFactor(factorPrediction);
            
            if nargin == 3
                obj.samplingUpdate.setNumSamplesByFactor(factorUpdate);
            else
                obj.samplingUpdate.setNumSamplesByFactor(factorPrediction);
            end
        end
        
        function setOnlineMode(obj, onlineMode)
            % Select between online and offline sampling.
            %
            % By default, the offline sample computation, i.e., the sample
            % cache, is used.
            %
            % Parameters:
            %   >> onlineMode (Logical scalar)
            %      If true, the samples will be computed online.
            %      Otherwise, samples from the sample cache will be used.
            
            obj.samplingPrediction.setOnlineMode(onlineMode);
            obj.samplingUpdate.setOnlineMode(onlineMode);
        end
        
        function setSymmetricMode(obj, useSymmetric)
            % Select between symmetric and asymmetric sampling.
            %
            % By default, the symmetric sampling mode is used.
            %
            % Parameters:
            %   >> useSymmetric (Logcial scalar)
            %      If true, the symmetric Gaussian LCD sampling scheme is used.
            %      Otherwise, the asymmetric one is used.
            
            obj.samplingPrediction.setSymmetricMode(useSymmetric);
            obj.samplingUpdate.setSymmetricMode(useSymmetric);
        end
    end
    
    properties (Access = 'private')
        % Gaussian LCD sampling used for prediction.
        samplingPrediction;
        
        % Gaussian LCD sampling used for update.
        samplingUpdate;
    end
end
