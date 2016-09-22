
classdef GaussianFilter < Filter
    % Abstract base class for Gaussian filters.
    %
    % This type of filter approximates the estimate of the system state as a Gaussian distribution.
    %
    % GaussianFilter Methods:
    %   GaussianFilter    - Class constructor.
    %   getName           - Get the filter name / description.
    %   setColor          - Set the filter color / plotting properties.
    %   getColor          - Get the current filter color / plotting properties.
    %   setState          - Set the system state.
    %   getState          - Get the current system state.
    %   getStateDim       - Get the dimension of the current system state.
    %   predict           - Perform a time update (prediction step).
    %   update            - Perform a measurement update (filter step) using the given measurement(s).
    %   step              - Perform a combined time and measurement update.
    %   getPointEstimate  - Get a point estimate of the current system state.
    %   setStateDecompDim - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim - Get the dimension of the unobservable part of the system state.
    
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
            obj.setStateDecompDim(0);
        end
        
        function setState(obj, state)
            if ~Checks.isClass(state, 'Distribution')
                obj.error('UnsupportedSystemState', ...
                          'state must be a subclass of Distribution.');
            end
            
            obj.dimState = state.getDimension();
            
            [obj.stateMean, obj.stateCov, obj.stateCovSqrt] = state.getMeanAndCovariance();
        end
        
        function state = getState(obj)
            state = Gaussian(obj.stateMean, obj.stateCov);
        end
        
        function [pointEstimate, uncertainty] = getPointEstimate(obj)
            % Get a point estimate of the current system state.
            %
            % Returns:
            %   << pointEstimate (Column vector)
            %      The current state mean.
            %
            %   << uncertainty (Positive definite matrix)
            %      The current state covariance.
            
            pointEstimate = obj.stateMean;
            uncertainty   = obj.stateCov;
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
    
    methods (Access = 'protected')
        function predictAnalytic(obj, sysModel)
            % Compute predicted state moments
            [predictedStateMean, ...
             predictedStateCov] = sysModel.analyticPredictedMoments(obj.stateMean, ...
                                                                    obj.stateCov);
            
            obj.checkPredictedMoments(predictedStateMean, predictedStateCov);
            
            obj.checkAndSavePrediction(predictedStateMean, predictedStateCov);
        end
        
        function checkAndSavePrediction(obj, predictedStateMean, predictedStateCov)
            % Check predicted state covariance is valid
            [isPosDef, predictedStateCovSqrt] = Checks.isCov(predictedStateCov);
            
            if ~isPosDef
                obj.warnIgnorePrediction('Predicted state covariance is not positive definite.');
                return;
            end
            
            % Save new state estimate
            obj.stateMean    = predictedStateMean;
            obj.stateCov     = predictedStateCov;
            obj.stateCovSqrt = predictedStateCovSqrt;
        end
        
        function observableStateDim = getObservableStateDim(obj)
            observableStateDim = obj.dimState - obj.stateDecompDim;
            
            % At least one variable of the system state must be "observable"
            if observableStateDim <= 0
                obj.error('InvalidUnobservableStateDimension', ...
                          'Invalid dimension of the unobservable part of the system state.');
            end
        end
        
        function [updatedStateMean, ...
                  updatedStateCov, ...
                  updatedStateCovSqrt] = decomposedStateUpdate(obj, updatedMean, updatedCov)
            % Update entire system state
            [updatedStateMean, ...
             updatedStateCov] = Utils.decomposedStateUpdate(obj.stateMean, obj.stateCov, ...
                                                            updatedMean, updatedCov);
            
            % Check updated state covariance is valid
            [isPosDef, updatedStateCovSqrt] = Checks.isCov(updatedStateCov);
            
            if ~isPosDef
                obj.ignoreMeas('Updated state covariance is not positive definite.');
            end
        end
    end
    
    methods (Access = 'private')
        function checkPredictedMoments(obj, mean, covariance)
            if ~Checks.isColVec(mean, obj.dimState)
                obj.error('InvalidPredictedStateMean', ...
                          ['Predicted state mean must be a ' ...
                           'column vector of dimension %d.'], ...
                          obj.dimState);
            end
            
            % Note: check for positive definiteness in GaussianFilter.checkAndSavePrediction()
            if ~Checks.isSquareMat(covariance, obj.dimState)
                obj.error('InvalidPredictedStateCovariance', ...
                          ['Predicted state covariance must be a ' ...
                           'positive definite matrix of dimension %dx%d.'], ...
                           obj.dimState, obj.dimState);
            end
        end
    end
    
    properties (Access = 'protected')
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
