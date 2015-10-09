
classdef BasePF < Filter
    % Abstract base class for Particle Filters (PFs).
    %
    % Provides utility methods for Particle Filters.
    %
    % BasePF Methods:
    %   BasePF           - Class constructor.
    %   getName          - Get the filter name / description.
    %   setColor         - Set the filter color / plotting properties.
    %   getColor         - Get the current filter color / plotting properties.
    %   setState         - Set the system state.
    %   getState         - Get the current system state.
    %   getStateDim      - Get the dimension of the current system state.
    %   predict          - Perform a time update (prediction step).
    %   update           - Perform a measurement update (filter step) using the given measurement(s).
    %   step             - Perform a combined time and measurement update.
    %   getPointEstimate - Get a point estimate of the current system state.
    
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
        function obj = BasePF(name)
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
            %   << obj (BasePF)
            %      A new BasePF instance.
            
            % Call superclass constructor
            obj = obj@Filter(name);
        end
    end
    
    methods (Access = 'protected')
        function predictedParticles = predictParticlesArbitraryNoise(obj, sysModel, particles, numParticles)
            % Sample system noise
            noise = sysModel.noise.drawRndSamples(numParticles);
            
            % Propagate particles and noise through system equation
            predictedParticles = sysModel.systemEquation(particles, noise);
            
            % Check predicted particles
            obj.checkPredictedStateSamples(predictedParticles, numParticles);
        end
        
        function predictedParticles = predictParticlesAdditiveNoise(obj, sysModel, particles, numParticles)
            % Sample additive system noise
            noise = sysModel.noise.drawRndSamples(numParticles);
            
            dimNoise = size(noise, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Propagate particles through system equation
            predictedParticles = sysModel.systemEquation(particles);
            
            % Check predicted particles
            obj.checkPredictedStateSamples(predictedParticles, numParticles);
            
            % Add system noise
            predictedParticles = predictedParticles + noise;
        end
        
        function predictedParticles = predictParticlesMixedNoise(obj, sysModel, particles, numParticles)
            % Sample system noise
            noise = sysModel.noise.drawRndSamples(numParticles);
            
            % Sample additive system noise
            addNoise = sysModel.additiveNoise.drawRndSamples(numParticles);
            
            dimAddNoise = size(addNoise, 1);
            
            obj.checkAdditiveSysNoise(dimAddNoise);
            
            % Propagate particles and noise through system equation
            predictedParticles = sysModel.systemEquation(particles, noise);
            
            % Check predicted particles
            obj.checkPredictedStateSamples(predictedParticles, numParticles);
            
            % Add system noise
            predictedParticles = predictedParticles + addNoise;
        end
        
        function values = evaluateLikelihood(obj, measModel, measurements, particles, numParticles)
            % Evaluate logarithmic likelihood
            logValues = measModel.logLikelihood(particles, measurements);
            
            obj.checkLogLikelihoodEvaluations(logValues, numParticles);
            
            % For numerical stability
            maxLogValue = max(logValues);
            
            logValues = logValues - maxLogValue;
            
            % Compute likelihohod values
            values = exp(logValues);
        end
    end
end
