
classdef GaussianFilter < Filter
    % Abstract base class for Gaussian filters.
    %
    % This type of filter approximates the estimate of the system state as a Gaussian distribution.
    %
    % GaussianFilter Methods:
    %   GaussianFilter            - Class constructor.
    %   copy                      - Copy a Filter instance.
    %   copyWithName              - Copy a Filter instance and give the copy a new name / description.
    %   getName                   - Get the filter name / description.
    %   setColor                  - Set the filter color / plotting properties.
    %   getColor                  - Get the current filter color / plotting properties.
    %   setState                  - Set the system state.
    %   getState                  - Get the current system state.
    %   getStateDim               - Get the dimension of the current system state.
    %   predict                   - Perform a time update (prediction step).
    %   update                    - Perform a measurement update (filter step) using the given measurement(s).
    %   step                      - Perform a combined time and measurement update.
    %   getPointEstimate          - Get a point estimate of the current system state.
    %   setStateDecompDim         - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim         - Get the dimension of the unobservable part of the system state.
    
    % Literature:
    %   Jannik Steinbring, Antonio Zea, Uwe D. Hanebeck,
    %   Semi-Analytic Progressive Gaussian Filtering,
    %   Proceedings of the 2016 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI),
    %   Baden-Baden, Germany, September 2016.
    %
    %   Tine Lefebvre, Herman Bruyninckx, Joris De Schutter,
    %   Nonlinear Kalman Filtering for Force-Controlled Robot Tasks,
    %   Appendix E: Partial Observation with the Kalman Filter,
    %   ser. Springer Tracts in Advanced Robotics.
    %   Berlin Heidelberg: Springer, 2005, vol. 19.
    
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
        function obj = GaussianFilter(name)
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
            %   << obj (GaussianFilter)
            %      A new GaussianFilter instance.
            
            % Call superclass constructor
            obj = obj@Filter(name);
            
            % By default, it is assumed that the entire system state is
            % required by the measurement model/likelihood function.
            obj.stateDecompDim = 0;
        end
        
        function state = getState(obj)
            state = Gaussian(obj.stateMean, obj.stateCov);
        end
        
        function [stateMean, stateCov] = getStateMeanAndCov(obj)
            stateMean = obj.stateMean;
            stateCov  = obj.stateCov;
        end
        
        function setStateDecompDim(obj, dim)
            % Set the dimension of the unobservable part of the system state.
            %
            % Consider a measurement model/likelihood function that only
            % depends on the subspace o of the entire system state [o, u]',
            % called the observable part. Due to the Gaussian state
            % estimate, the filter step can be divided into two parts.
            % First, the filter step of the respective filter is used to
            % only update the state estimate of the observable part o.
            % Second, the updated estimate of the entire state can now be
            % computed in closed-form based on the updated estimate for the
            % subspace o and the prior state estimate.
            %
            % For example, consider a 5D system state [a, b, c, d, e]'. If
            % the dimension of the unobservable part is set to two, the
            % observable part is o = [a, b, c]' and the unobservable part
            % is u = [d, e]'. That is, it is assumed that the unobservable
            % part comprises the last dimensions of the system state.
            %
            % A value of zero means that the entire system state is
            % required by the measurement model/likelihood function (i.e.,
            % the usual case).
            %
            % By default, the dimension of the unobservable part is set to
            % zero.
            %
            % Parameters:
            %   >> dim (Non-negative scalar)
            %      The new dimension of the unobservable part of the system state.
            
            if ~Checks.isNonNegativeScalar(dim)
                obj.error('InvalidDimension', ...
                          'dim must be a non-negative scalar.');
            end
            
            obj.stateDecompDim = ceil(dim);
        end
        
        function dim = getStateDecompDim(obj)
            % Get the dimension of the unobservable part of the system state.
            %
            % Returns:
            %   << dim (Non-negative scalar)
            %      Dimension of the unobservable part of the system state.
            
            dim = obj.stateDecompDim;
        end
    end
    
    methods (Abstract, Access = 'protected')
        [predictedStateMean, ...
         predictedStateCov] = predictedMomentsArbitraryNoise(obj, sysModel);
        
        [predictedStateMean, ...
         predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel);
        
        [updatedMean, ...
         updatedCov] = performUpdateObservable(obj, measModel, measurements, ...
                                               priorMean, priorCov, priorCovSqrt);
    end
    
    methods (Access = 'protected')
        function performSetState(obj, state)
            [obj.stateMean, obj.stateCov, obj.stateCovSqrt] = state.getMeanAndCov();
        end
        
        function performPrediction(obj, sysModel)
            if Checks.isClass(sysModel, 'LinearSystemModel')
                [predictedStateMean, ...
                 predictedStateCov] = obj.predictedMomentsLinear(sysModel);
            elseif Checks.isClass(sysModel, 'SystemModel')
                [predictedStateMean, ...
                 predictedStateCov] = obj.predictedMomentsArbitraryNoise(sysModel);
            elseif Checks.isClass(sysModel, 'AdditiveNoiseSystemModel')
                [predictedStateMean, ...
                 predictedStateCov] = obj.predictedMomentsAdditiveNoise(sysModel);
            elseif Checks.isClass(sysModel, 'MixedNoiseSystemModel')
                [predictedStateMean, ...
                 predictedStateCov] = obj.predictedMomentsMixedNoise(sysModel);
            else
                obj.errorSysModel('LinearSystemModel', ...
                                  'SystemModel', ...
                                  'AdditiveNoiseSystemModel', ...
                                  'MixedNoiseSystemModel');
            end
            
            obj.checkAndSavePrediction(predictedStateMean, predictedStateCov);
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsLinear(obj, sysModel)
            [predictedStateMean, ...
             predictedStateCov] = sysModel.analyticMoments(obj.stateMean, ...
                                                           obj.stateCov, ...
                                                           obj.stateCovSqrt);
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsMixedNoise(obj, sysModel)
            [addNoiseMean, addNoiseCov]  = sysModel.additiveNoise.getMeanAndCov();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimAddNoise);
            
            [mean, cov] = obj.predictedMomentsArbitraryNoise(sysModel);
            
            % Compute predicted state mean
            predictedStateMean = mean + addNoiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + addNoiseCov;
        end
        
        function checkAndSavePrediction(obj, predictedStateMean, predictedStateCov)
            % Check if predicted state covariance is valid
            predictedStateCovSqrt = obj.checkCovPrediction(predictedStateCov, 'Predicted state');
            
            % Save new state estimate
            obj.stateMean    = predictedStateMean;
            obj.stateCov     = predictedStateCov;
            obj.stateCovSqrt = predictedStateCovSqrt;
        end
        
        function performUpdate(obj, measModel, measurements)
            observableStateDim = obj.getObservableStateDim();
            
            % Use decomposed state update?
            if observableStateDim < obj.dimState
                % Extract observable part of the system state
                idx     = 1:observableStateDim;
                mean    = obj.stateMean(idx);
                cov     = obj.stateCov(idx, idx);
                covSqrt = obj.stateCovSqrt(idx, idx);
                
                % Update observable state variables
                [updatedMean, ...
                 updatedCov] = obj.performUpdateObservable(measModel, measurements, ...
                                                           mean, cov, covSqrt);
                
                % Check if updated observable state covariance is valid
                updatedCovSqrt = obj.checkCovUpdate(updatedCov, 'Updated observable state');
                
                % Update entire system state
                [updatedStateMean, ...
                 updatedStateCov] = Utils.decomposedStateUpdate(obj.stateMean, obj.stateCov, obj.stateCovSqrt, ...
                                                                updatedMean, updatedCov, updatedCovSqrt);
            else
                % Update entire system state
                [updatedStateMean, ...
                 updatedStateCov] = obj.performUpdateObservable(measModel, measurements, ...
                                                                obj.stateMean, obj.stateCov, obj.stateCovSqrt);
            end
            
            obj.checkAndSaveUpdate(updatedStateMean, updatedStateCov);
        end
        
        function observableStateDim = getObservableStateDim(obj)
            observableStateDim = obj.dimState - obj.stateDecompDim;
            
            % At least one variable of the system state must be "observable"
            if observableStateDim <= 0
                obj.error('InvalidUnobservableStateDimension', ...
                          'At least one variable of the system state must be "observable".');
            end
        end
        
        function checkAndSaveUpdate(obj, updatedStateMean, updatedStateCov)
            % Check if updated state covariance is valid
            updatedStateCovSqrt = obj.checkCovUpdate(updatedStateCov, 'Updated state');
            
            % Save new state estimate
            obj.stateMean    = updatedStateMean;
            obj.stateCov     = updatedStateCov;
            obj.stateCovSqrt = updatedStateCovSqrt;
        end
    end
    
    properties (GetAccess = 'protected', SetAccess = 'private')
        % Current system mean vector.
        stateMean;
        
        % Current system covariance matrix.
        stateCov;
        
        % Sqrt of the current system covariance matrix.
        stateCovSqrt;
    end
    
    properties (Access = 'private')
        % Dimension of the unobservable part of the system state.
        stateDecompDim;
    end
end
