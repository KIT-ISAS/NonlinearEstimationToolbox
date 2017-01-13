
classdef AnalyticKF < KF
    % Analytic Kalman Filter.
    %
    % AnalyticKF Methods:
    %   AnalyticKF                     - Class constructor.
    %   copy                           - Copy a Filter instance.
    %   copyWithName                   - Copy a Filter instance and give the copy a new name / description.
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
    %   setUseAnalyticSystemModel      - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel      - Get the current use of analytic moment calculation during a prediction.
    %   setStateDecompDim              - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim              - Get the dimension of the unobservable part of the system state.
    %   setUseAnalyticMeasurementModel - Enable or disable the use of analytic moment calculation during a filter step.
    %   getUseAnalyticMeasurementModel - Get the current use of analytic moment calculation during a filter step.
    %   setMaxNumIterations            - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations            - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold     - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold     - Get the current measurement validation threshold.
    %   getLastUpdateData              - Get information from the last performed measurement update.
    
    % Literature:
    %   Michael Roth, Gustaf Hendeby, and Fredrik Gustafsson,
    %   Nonlinear Kalman Filters Explained: A Tutorial on Moment Computations and Sigma Point Methods,
    %   Journal of Advances in Information Fusion, Vol. 11, No. 1, Jun 2016, pp. 47–70.
    %
    %   Marco F. Huber, Frederik Beutler, and Uwe D. Hanebeck,
    %   Semi-Analytic Gaussian Assumed Density Filter,
    %   Proceedings of the 2011 American Control Conference (ACC 2011), 2011.
    
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
        function obj = AnalyticKF(name)
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
            %      Default name: 'Analytic KF'.
            %
            % Returns:
            %   << obj (AnalyticKF)
            %      A new AnalyticKF instance.
            
            if nargin < 1
                name = 'Analytic KF';
            end
            
            % Superclass constructor
            obj = obj@KF(name);
        end
    end
    
    methods (Access = 'protected')
        function performPrediction(obj, sysModel)
            if Checks.isClass(sysModel, 'AnalyticSystemModel')
                [predictedStateMean, ...
                 predictedStateCov] = obj.predictedMomentsAnalytic(sysModel);
                
                obj.checkAndSavePrediction(predictedStateMean, predictedStateCov);
            else
                obj.errorSysModel('AnalyticSystemModel');
            end
        end
        
        function predictedMomentsArbitraryNoise(~, ~)
            % Dummy implementation
        end
        
        function predictedMomentsAdditiveNoise(~, ~)
            % Dummy implementation
        end
        
        function predictedMomentsMixedNoise(~, ~)
            % Dummy implementation
        end
        
        function [updatedMean, ...
                  updatedCov] = performUpdateObservable(obj, measModel, measurements, ...
                                                        priorMean, priorCov, priorCovSqrt)
            if Checks.isClass(measModel, 'AnalyticMeasurementModel')
                momentFunc = obj.getMomentFuncAnalytic(measModel, measurements);
                
                [updatedMean, ...
                 updatedCov] = obj.kalmanUpdate(measurements, momentFunc, ...
                                                priorMean, priorCov, priorCovSqrt);
            else
                obj.errorMeasModel('AnalyticMeasurementModel');
            end
        end
        
        function getMomentFuncArbitraryNoise(~, ~, ~)
            % Dummy implementation
        end
        
        function getMomentFuncAdditiveNoise(~, ~, ~)
            % Dummy implementation
        end
        
        function getMomentFuncMixedNoise(~, ~, ~)
            % Dummy implementation
        end
    end
end
