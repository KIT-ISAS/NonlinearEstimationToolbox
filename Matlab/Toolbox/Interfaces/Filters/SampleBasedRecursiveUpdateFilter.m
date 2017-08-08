
classdef SampleBasedRecursiveUpdateFilter < RecursiveUpdateFilter
    % Abstract base class for sample-based recursive update filters.
    %
    % SampleBasedRecursiveUpdateFilter Methods:
    %   SampleBasedRecursiveUpdateFilter - Class constructor.
    %   copy                             - Copy a Filter instance.
    %   copyWithName                     - Copy a Filter instance and give the copy a new name/description.
    %   getName                          - Get the filter name/description.
    %   setColor                         - Set the filter color/plotting properties.
    %   getColor                         - Get the filter color/plotting properties.
    %   setState                         - Set the system state.
    %   getState                         - Get the system state.
    %   getStateDim                      - Get the dimension of the system state.
    %   getStateMeanAndCov               - Get mean and covariance matrix of the system state.
    %   predict                          - Perform a state prediction.
    %   update                           - Perform a measurement update.
    %   step                             - Perform a combined state prediction and measurement update.
    %   setStateDecompDim                - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim                - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing      - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing      - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing          - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing          - Get the post-processing method for the measurement update.
    %   setMeasGatingThreshold           - Set the measurement gating threshold.
    %   getMeasGatingThreshold           - Get the measurement gating threshold.
    %   setNumRecursionSteps             - Set the number of recursion steps that are performed by a measurement update.
    %   getNumRecursionSteps             - Get the number of recursion steps that are performed by a measurement update.
    
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
    
    % Literature:
    %   Yulong Huang, Yonggang Zhang, Ning Li, and Lin Zhao,
    %   Design of Sigma-Point Kalman Filter with Recursive Updated Measurement,
    %   Circuits, Systems, and Signal Processing, pp. 1-16, Aug. 2015.
    
    methods
        function obj = SampleBasedRecursiveUpdateFilter(name)
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
            %   << obj (SampleBasedRecursiveUpdateFilter)
            %      A new SampleBasedRecursiveUpdateFilter instance.
            
            % Call superclass constructor
            obj = obj@RecursiveUpdateFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function [measMean, measCov, ...
                  stateMeasCrossCov, R, H] = getRecursionData(obj, stateMean, ~, stateCovSqrt)
            [measMean, measCov, ...
             stateMeasCrossCov] = obj.measMomentFunc(stateMean, stateCovSqrt);
            
            A = stateMeasCrossCov' / stateCovSqrt';
            
            R = measCov - A * A';
            
            if nargout == 5
                H = A / stateCovSqrt;
            end
        end
    end
    
    methods (Abstract, Access = 'protected')
        [measMean, measCov, ...
         stateMeasCrossCov] = measMomentFunc(obj, priorMean, priorCovSqrt);
    end
end
