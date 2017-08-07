
classdef RecursiveUpdateFilter < LinearGaussianFilter
    % Abstract base class for recursive update filters.
    %
    % RecursiveUpdateFilter Methods:
    %   RecursiveUpdateFilter       - Class constructor.
    %   copy                        - Copy a Filter instance.
    %   copyWithName                - Copy a Filter instance and give the copy a new name/description.
    %   getName                     - Get the filter name/description.
    %   setColor                    - Set the filter color/plotting properties.
    %   getColor                    - Get the filter color/plotting properties.
    %   setState                    - Set the system state.
    %   getState                    - Get the system state.
    %   getStateDim                 - Get the dimension of the system state.
    %   getStateMeanAndCov          - Get mean and covariance matrix of the system state.
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
    %   setNumIterations            - Set the number of iterations that will be performed by a measurement update.
    %   getNumIterations            - Get the number of iterations that will be performed by a measurement update.
    
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
    
    methods (Sealed)
        function obj = RecursiveUpdateFilter(name)
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
            %   << obj (RecursiveUpdateFilter)
            %      A new RecursiveUpdateFilter instance.
            
            % Call superclass constructor
            obj = obj@LinearGaussianFilter(name);
            
            % By default, 10 iterations are performed by a measurement update.
            obj.numIterations = 10;
        end
        
        function setNumIterations(obj, numIterations)
            % Set the number of iterations that will be performed by a measurement update.
            %
            % By default, 10 iterations are performed by a measurement update.
            %
            % Parameters:
            %   >> numIterations (Positive scalar)
            %      The new number of iterations that will be performed by a measurement update.
            
            if ~Checks.isPosScalar(numIterations)
                obj.error('InvalidNumberOfIterations', ...
                          'numIterations must be a positive scalar.');
            end
            
            obj.numIterations = ceil(numIterations);
        end
        
        function numIterations = getNumIterations(obj)
            % Get the number of iterations that will be performed by a measurement update.
            %
            % Returns:
            %   << numIterations (Scalar)
            %      The number of iterations that will be performed by a measurement update.
            
            numIterations = obj.numIterations;
        end
    end
    
    methods (Sealed, Access = 'protected')
        function [updatedMean, ...
                  updatedCov] = updateNonlinear(obj, measurement, ...
                                                priorMean, priorCov, priorCovSqrt)
            % First recursion step
            [measMean, measCov, ...
             stateMeasCrossCov, R] = obj.getRecursionData(priorMean, priorCov, priorCovSqrt);
            
            measCovSqrt = obj.checkCovUpdate(measCov, 'Measurement');
            
            r = 1 / obj.numIterations;
            A = stateMeasCrossCov / measCovSqrt';
            K = r * (A / measCovSqrt);
            
            innovation      = measurement - measMean;
            updatedMean     = priorMean + K * innovation;
            updatedCov      = priorCov + ((r - 2) * r) * (A * A');
            updatedCrossCov = -K * R;
            
            if obj.isMeasGatingEnabled()
                % Perform measurement gating before any measurement information is processed
                dimMeas = size(measurement, 1);
                
                t = measCovSqrt \ innovation;
                sqMeasMahalDist = t' * t;
                
                try
                    obj.measurementGating(dimMeas, sqMeasMahalDist);
                catch ex
                    obj.ignoreMeas(ex.message);
                end
            end
            
            % Next recursion steps
            for i = 2:obj.numIterations
                % Check if intermediate state covariance matrix is valid
                updatedCovSqrt = obj.checkCovUpdate(updatedCov, 'Intermediate state');
                
                [measMean, measCov, ...
                 stateMeasCrossCov, R, H] = obj.getRecursionData(updatedMean, updatedCov, updatedCovSqrt);
                
                M = H * updatedCrossCov;
                
                measCov           = measCov + (M + M');
                stateMeasCrossCov = stateMeasCrossCov + updatedCrossCov;
                
                measCovSqrt = obj.checkCovUpdate(measCov, 'Measurement');
                
                r = 1 / (obj.numIterations + 1 - i);
                A = stateMeasCrossCov / measCovSqrt';
                K = r * (A / measCovSqrt);
                
                innovation      = measurement - measMean;
                updatedMean     = updatedMean + K * innovation;
                updatedCov      = updatedCov + ((r - 2) * r) * (A * A');
                updatedCrossCov = updatedCrossCov - K * (R + M);
            end
        end
    end
    
    methods (Abstract, Access = 'protected')
        [measMean, measCov, ...
         stateMeasCrossCov, R, H] = getRecursionData(obj, stateMean, stateCov, stateCovSqrt);
    end
    
    properties (Access = 'private')
        % The number of iterations that will be performed by a measurement update.
        numIterations;
    end
end
