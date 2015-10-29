
classdef SIRPF < PF
    % The Sequential Importance Resampling Particle Filter (SIRPF).
    %
    % SIRPF Methods:
    %   SIRPF                      - Class constructor.
    %   getName                    - Get the filter name / description.
    %   setColor                   - Set the filter color / plotting properties.
    %   getColor                   - Get the current filter color / plotting properties.
    %   setState                   - Set the system state.
    %   getState                   - Get the current system state.
    %   getStateDim                - Get the dimension of the current system state.
    %   predict                    - Perform a time update (prediction step).
    %   update                     - Perform a measurement update (filter step) using the given measurement(s).
    %   step                       - Perform a combined time and measurement update.
    %   getPointEstimate           - Get a point estimate of the current system state.
    %   setNumParticles            - Set the number of particles used by the filter.
    %   getNumParticles            - Get the current number of particles used by the filter.
    %   setMinAllowedNormalizedESS - Set the minimum allowed normalized effective sample size (ESS).
    %   getMinAllowedNormalizedESS - Get the current minimum allowed normalized effective sample size (ESS).
    
    % Literature:
    %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
    %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
    %   Artech House Publishers, 2004.
    
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
        function obj = SIRPF(name)
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
            %      Default name: 'SIR-PF'.
            %
            % Returns:
            %   << obj (SIRPF)
            %      A new SIRPF instance.
            
            if nargin < 1
                name = 'SIR-PF';
            end
            
            % Superclass constructor
            obj = obj@PF(name);
        end
    end
    
    methods (Access = 'protected')
        function performPrediction(obj, sysModel)
            % Implements the prediction described in:
            %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
            %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
            %   Artech House Publishers, 2004,
            %   Section 3.5.1
            
            if Checks.isClass(sysModel, 'SystemModel')
                obj.particles = obj.predictParticlesArbitraryNoise(sysModel, obj.particles, obj.numParticles);
            elseif Checks.isClass(sysModel, 'AdditiveNoiseSystemModel')
                obj.particles = obj.predictParticlesAdditiveNoise(sysModel, obj.particles, obj.numParticles);
            elseif Checks.isClass(sysModel, 'MixedNoiseSystemModel')
                obj.particles = obj.predictParticlesMixedNoise(sysModel, obj.particles, obj.numParticles);
            else
                obj.errorSysModel('SystemModel', ...
                                  'AdditiveNoiseSystemModel', ...
                                  'MixedNoiseSystemModel');
            end
        end
        
        function performUpdate(obj, measModel, measurements)
            if Checks.isClass(measModel, 'Likelihood')
                obj.updateLikelihood(measModel, measurements);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
        
        function updateLikelihood(obj, measModel, measurements)
            % Implements the measurement update described in:
            %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
            %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
            %   Artech House Publishers, 2004,
            %   Section 3.5.1
            
            % Evaluate likelihood
            values = obj.evaluateLikelihood(measModel, measurements, obj.particles, obj.numParticles);
            
            % Multiply prior weights with likelihood evaluations
            values = values .* obj.weights;
            
            % Normalize particle weights
            sumWeights = sum(values);
            
            if sumWeights <= 0
                obj.warnIgnoreMeas('Sum of computed posterior particle weights is not positive.');
                return;
            end
            
            % Save new particle weights
            obj.weights = values / sumWeights;
            
            % Resample if necessary
            normalizedESS = obj.getNormalizedESS();
            
            if normalizedESS < obj.minAllowedNormalizedESS
                obj.resample();
            end
        end
    end
end
