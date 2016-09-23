
classdef CGPF < GPF
    % The Gaussian Particle Filter (GPF) with a combined time and measurement update.
    %
    % Use the step() method to perform the combined time and measurement update.
    % Note that the separated predictions and updates of the GPF are still possible
    % by simply using predict() and update(), respectively (e.g., in case where a
    % prediction is required but no measurement is available).
    %
    % CGPF Methods:
    %   CGPF                      - Class constructor.
    %   getName                   - Get the filter name / description.
    %   setColor                  - Set the filter color / plotting properties.
    %   getColor                  - Get the current filter color / plotting properties.
    %   setState                  - Set the system state.
    %   getState                  - Get the current system state.
    %   getStateDim               - Get the dimension of the current system state.
    %   predict                   - Perform a time update (prediction step).
    %   update                    - Perform a measurement update (filter step) using the given measurement(s).
    %   step                      - Perform a combined time and measurement update.
    %   getPointEstimate          - Get a point estimate of the current system state.
    %   setUseAnalyticSystemModel - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel - Get the current use of analytic moment calculation during a prediction.
    %   setStateDecompDim         - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim         - Get the dimension of the unobservable part of the system state.
    %   setNumParticles           - Set the number of particles used by the filter.
    %   getNumParticles           - Get the current number of particles used by the filter.
    
    % Literature:
    %   Jayesh H. Kotecha and Petar M. Djuric,
    %   Gaussian Particle Filtering,
    %   IEEE Transactions on Signal Processing
    %   Vol. 51, No. 10, pages 2592 - 2601, October 2003
    
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
        function performStep(obj, sysModel, measModel, measurements)
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
                obj.updateLikelihoodCombined(measModel, measurements, predictedParticles);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
        
        function updateLikelihoodCombined(obj, measModel, measurements, particles)
            % It is important to note that for the combined time and
            % measurement update the predicted state estimate is NOT
            % necessarily Gaussian. Hence, we cannot use the decomposed
            % state update even if the likelihood function does not depend
            % upon the entire system state.
            
            % Standard GPF update
            [updatedStateMean, ...
             updatedStateCov] = obj.updateLikelihoodObservable(measModel, measurements, particles);
            
            obj.checkAndSaveUpdate(updatedStateMean, updatedStateCov);
        end
    end
end
