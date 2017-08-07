
classdef SampleBasedIterativeKalmanFilter < IterativeKalmanFilter
    % Abstract base class for sample-based iterative Kalman filters.
    %
    % SampleBasedIterativeKalmanFilter Methods:
    %   SampleBasedIterativeKalmanFilter - Class constructor.
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
    %   setMaxNumIterations              - Set the maximum number of iterations that will be performed by a measurement update.
    %   getMaxNumIterations              - Get the maximum number of iterations that will be performed by a measurement update.
    %   getNumIterations                 - Get number of iterations performed by the last measurement update.
    %   setConvergenceCheck              - Set a convergence check to determine if no further iterations are required.
    %   getConvergenceCheck              - Get the convergence check.
    
    % Literature:
    %   Ángel F. Garcı́a-Fernández, Lennart Svensson, Mark Morelande, and Simo Särkkä,
    %   Posterior Linearisation Filter: Principles and Implementation Using Sigma Points,
    %   IEEE Transactions on Signal Processing, vol. 63, no. 20, pp. 5561-5573, Oct. 2015.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2017  Jannik Steinbring <jannik.steinbring@kit.edu>
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
        function obj = SampleBasedIterativeKalmanFilter(name)
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
            %   << obj (SampleBasedIterativeKalmanFilter)
            %      A new SampleBasedIterativeKalmanFilter instance.
            
            % Call superclass constructor
            obj = obj@IterativeKalmanFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getMeasMoments(obj, priorMean, ~, priorCovSqrt)
            [measMean, measCov, ...
             stateMeasCrossCov] = obj.measMomentFunc(priorMean, priorCovSqrt);
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getIterationMeasMoments(obj, priorMean, priorCov, priorCovSqrt, ...
                                                               updatedMean, ~, updatedCovSqrt)
            [measMean, measCov, ...
             stateMeasCrossCov] = obj.measMomentFunc(updatedMean, updatedCovSqrt);
            
            A = stateMeasCrossCov' / updatedCovSqrt';
            H = A / updatedCovSqrt;
            P = H * priorCovSqrt;
            
            measMean          = measMean + H * (priorMean - updatedMean);
            measCov           = measCov + P * P' - A * A';
            stateMeasCrossCov = priorCov * H';
        end
    end
    
    methods (Abstract, Access = 'protected')
        [measMean, measCov, ...
         stateMeasCrossCov] = measMomentFunc(obj, priorMean, priorCovSqrt);
    end
end
