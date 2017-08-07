
classdef S2RUF < SampleBasedRecursiveUpdateFilter & SmartSamplingLinearGaussianFilter
    % The smart sampling recursive update filter (S²RUF)
    %
    % S2RUF Methods:
    %   S2RUF                         - Class constructor.
    %   copy                          - Copy a Filter instance.
    %   copyWithName                  - Copy a Filter instance and give the copy a new name/description.
    %   getName                       - Get the filter name/description.
    %   setColor                      - Set the filter color/plotting properties.
    %   getColor                      - Get the filter color/plotting properties.
    %   setState                      - Set the system state.
    %   getState                      - Get the system state.
    %   getStateDim                   - Get the dimension of the system state.
    %   getStateMeanAndCov            - Get mean and covariance matrix of the system state.
    %   predict                       - Perform a state prediction.
    %   update                        - Perform a measurement update.
    %   step                          - Perform a combined state prediction and measurement update.
    %   setStateDecompDim             - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim             - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing   - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing   - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing       - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing       - Get the post-processing method for the measurement update.
    %   setMeasGatingThreshold        - Set the measurement gating threshold.
    %   getMeasGatingThreshold        - Get the measurement gating threshold.
    %   setNumIterations              - Set the number of iterations that will be performed by a measurement update.
    %   getNumIterations              - Get the number of iterations that will be performed by a measurement update.
    %   setNumSamples                 - Set absolute numbers of samples used for state prediction and measurement update.
    %   setNumSamplesByFactors        - Set linear factors to determine the number of samples used for state prediction and measurement update.
    %   getNumSamplesConfigPrediction - Get the number of samples configuration used for the state prediction.
    %   getNumSamplesConfigUpdate     - Get the number of samples configuration used for the measurement update.
    
    % Literature:
    %   Jannik Steinbring, Martin Pander, and Uwe D. Hanebeck,
    %   The Smart Sampling Kalman Filter with Symmetric Samples
    %   Journal of Advances in Information Fusion, Vol. 11, No. 1, Jun 2016, pp. 71-90.
    %
    %   Yulong Huang, Yonggang Zhang, Ning Li, and Lin Zhao,
    %   Design of Sigma-Point Kalman Filter with Recursive Updated Measurement,
    %   Circuits, Systems, and Signal Processing, Aug 2015, pp. 1-16.
    
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
        function obj = S2RUF(name)
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
            %      Default name: 'S2RUF'.
            %
            % Returns:
            %   << obj (S2RUF)
            %      A new S2RUF instance.
            
            if nargin < 1
                name = 'S2RUF';
            end
            
            % Call superclass constructors
            obj = obj@SampleBasedRecursiveUpdateFilter(name);
            obj = obj@SmartSamplingLinearGaussianFilter(name);
        end
    end
end
