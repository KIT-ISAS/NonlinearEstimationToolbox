
classdef TaylorBasedIterativeKalmanFilter < IterativeKalmanFilter
    % Abstract base class for iterative Kalman filters that are based on Taylor series expansion.
    %
    % TaylorBasedIterativeKalmanFilter Methods:
    %   TaylorBasedIterativeKalmanFilter - Class constructor.
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
    %   Dan Simon,
    %   Optimal State Estimation,
    %   Sections 13.3,
    %   1st ed. Wiley & Sons, 2006.
    
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
        function obj = TaylorBasedIterativeKalmanFilter(name)
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
            %   << obj (TaylorBasedIterativeKalmanFilter)
            %      A new TaylorBasedIterativeKalmanFilter instance.
            
            % Call superclass constructor
            obj = obj@IterativeKalmanFilter(name);
        end
    end
    
    methods (Sealed, Access = 'protected')
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getMeasMoments(obj, priorMean, priorCov, priorCovSqrt)
            [h, H, R] = obj.linearizedModel(priorMean, priorCov, priorCovSqrt);
            
            P = H * priorCovSqrt;
            
            measMean          = h;
            measCov           = P * P' + R;
            stateMeasCrossCov = priorCov * H';
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = getIterationMeasMoments(obj, priorMean, priorCov, priorCovSqrt, ...
                                                               updatedMean, updatedCov, updatedCovSqrt)
            [h, H, R] = obj.linearizedModel(updatedMean, updatedCov, updatedCovSqrt);
            
            P = H * priorCovSqrt;
            
            measMean          = h + H * (priorMean - updatedMean);
            measCov           = P * P' + R;
            stateMeasCrossCov = priorCov * H';
        end
    end
    
    methods (Abstract, Access = 'protected')
        [h, H, R] = linearizedModel(obj, stateMean, stateCov, stateCovSqrt);
    end
end
