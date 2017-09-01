
classdef CGPF < GPF
    % The Gaussian particle filter with a combined time and measurement update.
    %
    % Use the step() method to perform the combined time and measurement update.
    % Note that the separated predictions and updates of the GPF are still possible
    % by simply using predict() and update(), respectively (e.g., in case where a
    % prediction is required but no measurement is available).
    %
    % CGPF Methods:
    %   CGPF                        - Class constructor.
    %   copy                        - Copy a Filter instance.
    %   copyWithName                - Copy a Filter instance and give the copy a new name/description.
    %   getName                     - Get the filter name/description.
    %   setColor                    - Set the filter color/plotting properties.
    %   getColor                    - Get the filter color/plotting properties.
    %   setState                    - Set the system state.
    %   getState                    - Get the system state.
    %   setStateMeanAndCov          - Set the system state by means of mean and covariance matrix.
    %   getStateMeanAndCov          - Get mean and covariance matrix of the system state.
    %   getStateDim                 - Get the dimension of the system state.
    %   predict                     - Perform a state prediction.
    %   update                      - Perform a measurement update.
    %   step                        - Perform a combined state prediction and measurement update.
    %   setStateDecompDim           - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim           - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing     - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing     - Get the post-processing method for the measurement update.
    %   setNumParticles             - Set the number of particles used by the filter.
    %   getNumParticles             - Get the number of particles used by the filter.
    
    % Literature:
    %   Jayesh H. Kotecha and Petar M. Djuric,
    %   Gaussian Particle Filtering,
    %   IEEE Transactions on Signal Processing, vol. 51, no. 10, pp. 2592-2601, Oct. 2003.
    
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
    
    methods
        function obj = CGPF(name)
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
            %      Default name: 'CGPF'.
            %
            % Returns:
            %   << obj (CGPF)
            %      A new CGPF instance.
            
            if nargin < 1
                name = 'CGPF';
            end
            
            % Call superclass constructor
            obj = obj@GPF(name);
        end
    end
    
    methods (Access = 'protected')
        function performStep(obj, sysModel, measModel, measurement)
            particles = obj.getStateParticles();
            
            if Checks.isClass(sysModel, 'SystemModel')
                predictedParticles = obj.predictParticlesArbitraryNoise(sysModel, particles, obj.numParticles);
            elseif Checks.isClass(sysModel, 'AdditiveNoiseSystemModel')
                predictedParticles = obj.predictParticlesAdditiveNoise(sysModel, particles, obj.numParticles);
            elseif Checks.isClass(sysModel, 'MixedNoiseSystemModel')
                predictedParticles = obj.predictParticlesMixedNoise(sysModel, particles, obj.numParticles);
            else
                obj.errorSysModel('SystemModel', ...
                                  'AdditiveNoiseSystemModel', ...
                                  'MixedNoiseSystemModel');
            end
            
            if Checks.isClass(measModel, 'Likelihood')
                % It is important to note that for the combined time and
                % measurement update the predicted state estimate is NOT
                % necessarily Gaussian. Hence, we cannot use the decomposed
                % state update even if the likelihood function does not depend
                % upon the entire system state.
                
                % Standard GPF update
                [updatedStateMean, ...
                 updatedStateCov] = obj.updateLikelihood(measModel, measurement, predictedParticles);
                
                obj.checkAndSaveUpdate(updatedStateMean, updatedStateCov);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
    end
end
