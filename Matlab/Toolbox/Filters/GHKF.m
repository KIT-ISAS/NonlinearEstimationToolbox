
classdef GHKF < LRKF
    % The Gauss-Hermite Kalman Filter (GHKF).
    %
    % GHKF Methods:
    %   GHKF                           - Class constructor.
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
    %   setMaxNumIterations            - Set the maximum number of iterations that will be performed during a measurement update.
    %   getMaxNumIterations            - Get the current maximum number of iterations that will be performed during a measurement update.
    %   setMeasValidationThreshold     - Set a threshold to perform a measurement validation (measurement acceptance/rejection).
    %   getMeasValidationThreshold     - Get the current measurement validation threshold.
    %   getLastUpdateData              - Get information from the last performed measurement update.
    %   setUseAnalyticMeasurementModel - Enable or disable the use of analytic moment calculation during a filter step.
    %   getUseAnalyticMeasurementModel - Get the current use of analytic moment calculation during a filter step.
    %   setNumQuadraturePoints         - Set the number of quadrature points.
    %   getNumQuadraturePoints         - Get the current number of quadrature points.
    
    % Literature:
    %   Kazufumi Ito, Kaiqi Xiong,
    %   Gaussian Filters for Nonlinear Filtering Problems,
    %   IEEE Transactions on Automatic Control, Vol. 45, Issue 5, Pages 910 - 927, May, 2000.
    
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
        function obj = GHKF(name)
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
            %      Default name: 'GHKF'.
            %
            % Returns:
            %   << obj (GHKF)
            %      A new GHKF instance.
            
            if nargin < 1
                name = 'GHKF';
            end
            
            sampling = GaussianSamplingGHQ();
            
            obj = obj@LRKF(name, sampling);
            
            obj.ghqSampling = sampling;
        end
        
        function setNumQuadraturePoints(obj, numPoints)
            % Set the number of quadrature points.
            %
            % By default, 2 quadrature points are used.
            %
            % Parameters:
            %   >> numPoints (Scalar in { 2, 3, 4 })
            %      The new number of quadrature points.
            
            obj.ghqSampling.setNumQuadraturePoints(numPoints);
        end
        
        function numPoints = getNumQuadraturePoints(obj)
            % Get the current number of quadrature points.
            %
            % Returns:
            %   << numPoints (Scalar in { 2, 3, 4 })
            %      The current number of quadrature points.
            
            numPoints = obj.ghqSampling.getNumQuadraturePoints();
        end
    end
    
    properties (Access = 'private')
        % Gaussian sampling used for prediction and update.
        ghqSampling;
    end
end
