
classdef GaussHermiteLinearGaussianFilter < SampleBasedLinearGaussianFilter
    % Abstract base class for linear Gaussian filters that are based on the
    % Gauss-Hermite quadrature Gaussian sampling technique.
    %
    % GaussHermiteLinearGaussianFilter Methods:
    %   GaussHermiteLinearGaussianFilter - Class constructor.
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
    %   setNumQuadraturePoints           - Set the number of quadrature points used for state prediction and measurement update.
    %   getNumQuadraturePoints           - Get the number of quadrature points used for state prediction and measurement update.
    
    % Literature:
    %   Kazufumi Ito and Kaiqi Xiong,
    %   Gaussian Filters for Nonlinear Filtering Problems,
    %   IEEE Transactions on Automatic Control, vol. 45, no. 5, pp. 910-927, May 2000.
    
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
        function obj = GaussHermiteLinearGaussianFilter(name)
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
            %   << obj (GaussHermiteLinearGaussianFilter)
            %      A new GaussHermiteLinearGaussianFilter instance.
            
            samplingPred = GaussianSamplingGHQ();
            samplingUp   = GaussianSamplingGHQ();
            
            % Call superclass constructor
            obj = obj@SampleBasedLinearGaussianFilter(name, samplingPred, samplingUp);
            
            % By default, 2 quadrature points are used.
            obj.samplingPrediction.setNumQuadraturePoints(2);
            obj.samplingUpdate.setNumQuadraturePoints(2);
        end
        
        function setNumQuadraturePoints(obj, numPointsPrediction, numPointsUpdate)
            % Set the number of quadrature points used for state prediction and measurement update.
            %
            % By default, 2 quadrature points are used for both state
            % prediction and measurement update.
            %
            % Parameters:
            %   >> numPointsPrediction (Scalar in { 2, 3, 4 })
            %      The new number of quadrature points used for the state prediction.
            %
            %   >> numPointsUpdate (Scalar in { 2, 3, 4 })
            %      The new number of quadrature points used for the measurement update.
            %      If nothing is passed, the number of quadrature points specified
            %      for the state prediction is also used for the measurement update.
            
            obj.samplingPrediction.setNumQuadraturePoints(numPointsPrediction);
            
            if nargin == 3
                obj.samplingUpdate.setNumQuadraturePoints(numPointsUpdate);
            else
                obj.samplingUpdate.setNumQuadraturePoints(numPointsPrediction);
            end
        end
        
        function [numPointsPrediction, ...
                  numPointsUpdate] = getNumQuadraturePoints(obj)
            % Get the number of quadrature points used for state prediction and measurement update.
            %
            % Returns:
            %   << numPointsPrediction (Scalar in { 2, 3, 4 })
            %      The number of quadrature points used for the state prediction.
            %
            %   << numPointsUpdate (Scalar in { 2, 3, 4 })
            %      The number of quadrature points used for the measurement update.
            
            numPointsPrediction = obj.samplingPrediction.getNumQuadraturePoints();
            numPointsUpdate     = obj.samplingUpdate.getNumQuadraturePoints();
        end
    end
end
