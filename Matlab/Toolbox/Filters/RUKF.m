
classdef RUKF < LRKF
    % The Randomized Unscented Kalman Filter (RUKF).
    %
    % RUKF Methods:
    %   RUKF                           - Class constructor.
    %   getName                        - Get the filter name / description.
    %   setColor                       - Set the filter color / plotting properties.
    %   getColor                       - Get the current filter color / plotting properties.
    %   setState                       - Set the system state.
    %   getState                       - Get the current system state.
    %   getStateDim                    - Get the dimension of the current system state.
    %   predict                        - Perform a time update (prediction step).
    %   update                         - Perform a measurement update (filter step) using the given measurement(s).
    %   getPointEstimate               - Get a point estimate of the current system state.
    %   setMaxNumIterations            - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations            - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold     - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold     - Get the current measurement validation threshold.
    %   getLastUpdateData              - Get information from the last performed measurement update.
    %   setUseAnalyticSystemModel      - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel      - Get the current use of analytic moment calculation during a prediction.
    %   setUseAnalyticMeasurementModel - Enable or disable the use of analytic moment calculation during a filter step.
    %   getUseAnalyticMeasurementModel - Get the current use of analytic moment calculation during a filter step.
    %   setNumIterations               - Set the number of iterations used for prediction and update.
    %   getNumIterations               - Get the current number of iterations used for prediction and update.
    
    % Literature:
    %   Jindrich Dunik, Ondrej Straka, and Miroslav Simandl,
    %   The Development of a Randomised Unscented Kalman Filter,
    %   Proceedings of the 18th IFAC World Congress, Milano, Italy, Aug. 2011, pp. 8-13.
    %
    %   Jindrich Dunik, Ondrej Straka, and Miroslav Simandl,
    %   Stochastic Integration Filter,
    %   IEEE Transactions on Automatic Control Vol. 58, No.6, June 2013, pp. 1561-1566.
    
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
        function obj = RUKF(name)
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
            %      Default name: 'RUKF'.
            %
            % Returns:
            %   << obj (RUKF)
            %      A new RUKF instance.
            
            if nargin < 1
                name = 'RUKF';
            end
            
            samplingPred = GaussianSamplingRUKF();
            samplingUp   = GaussianSamplingRUKF();
            
            obj = obj@LRKF(name, samplingPred, samplingUp);
            
            obj.samplingPrediction = samplingPred;
            obj.samplingUpdate     = samplingUp;
            
            % By default, 5 iterations are used for prediction and update.
            obj.setNumIterations(5);
        end
        
        function setNumIterations(obj, numIterationsPrediction, numIterationsUpdate)
            % Set the number of iterations used for prediction and update.
            %
            % By default, 5 iterations are used for prediction and update.
            %
            % Parameters:
            %   >> numIterationsPrediction (Positive scalar)
            %      The new number of iterations used for the prediction.
            %
            %   >> numIterationsUpdate (Positive scalar)
            %      The new number of iterations used for the update.
            %      Default: the same number of iterations specified for the prediction.
            
            obj.samplingPrediction.setNumIterations(numIterationsPrediction);
            
            if nargin == 3
                obj.samplingUpdate.setNumIterations(numIterationsUpdate);
            else
                obj.samplingUpdate.setNumIterations(numIterationsPrediction);
            end
        end
        
        function [numIterationsPrediction, numIterationsUpdate] = getNumIterations(obj)
            % Get the current number of iterations for prediction and update.
            %
            % Returns:
            %   << numIterationsPrediction (Positive scalar)
            %      The current number of iterations used for the prediction.
            %
            %   << numIterationsUpdate (Positive scalar)
            %      The current number of iterations used for the update.
            
            numIterationsPrediction = obj.samplingPrediction.getNumIterations();
            numIterationsUpdate     = obj.samplingUpdate.getNumIterations();
        end
    end
    
    properties (Access = 'private')
        % Gaussian sampling used for prediction.
        samplingPrediction;
        
        % Gaussian sampling used for update.
        samplingUpdate;
    end
end
