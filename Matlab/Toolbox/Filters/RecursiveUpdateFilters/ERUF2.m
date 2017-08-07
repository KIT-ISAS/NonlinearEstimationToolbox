
classdef ERUF2 < TaylorBasedRecursiveUpdateFilter & SecondOrderTaylorLinearGaussianFilter
    % The second-order extended recursive update filter (ERUF2).
    %
    % ERUF2 Methods:
    %   ERUF2                       - Class constructor.
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
    %   Michael Athans, Richard P. Wishner, and Anthony Bertolini,
    %   Suboptimal State Estimation for Continuous-Time Nonlinear Systems from Discrete Noisy Measurements,
    %   IEEE Transactions on Automatic Control, vol. 13, no. 5, pp. 504-514, Oct. 1968.
    %
    %   Renato Zanetti,
    %   Recursive Update Filtering for Nonlinear Estimation,
    %   IEEE Transactions on Automatic Control, vol. 57, no. 6, pp. 1481-1490, Jun. 2012.
    
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
        function obj = ERUF2(name)
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
            %      Default name: 'ERUF2'.
            %
            % Returns:
            %   << obj (ERUF2)
            %      A new ERUF2 instance.
            
            if nargin < 1
                name = 'ERUF2';
            end
            
            % Call superclass constructors
            obj = obj@TaylorBasedRecursiveUpdateFilter(name);
            obj = obj@SecondOrderTaylorLinearGaussianFilter(name);
        end
    end
end
