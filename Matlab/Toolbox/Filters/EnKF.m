
classdef EnKF < ParticleFilter
    % The ensemble Kalman filter (EnKF).
    %
    % EnKF Methods:
    %   EnKF               - Class constructor.
    %   copy               - Copy a Filter instance.
    %   copyWithName       - Copy a Filter instance and give the copy a new name/description.
    %   getName            - Get the filter name/description.
    %   setColor           - Set the filter color/plotting properties.
    %   getColor           - Get the filter color/plotting properties.
    %   setState           - Set the system state.
    %   getState           - Get the system state.
    %   getStateDim        - Get the dimension of the system state.
    %   getStateMeanAndCov - Get mean and covariance matrix of the system state.
    %   predict            - Perform a state prediction.
    %   update             - Perform a measurement update.
    %   step               - Perform a combined state prediction and measurement update.
    %   setEnsembleSize    - Set the size of the ensemble (i.e., the number of samples).
    %   getEnsembleSize    - Get the ensemble size of the filter.
    
    % Literature:
    %   S. Gillijns, O. Barrero Mendoza, J. Chandrasekar, B. L. R. De Moor, D. S. Bernstein, and A. Ridley,
    %   What Is the Ensemble Kalman Filter and How Well Does It Work?,
    %   Proceedings of the 2006 American Control Conference (ACC 2006),
    %   Minneapolis, USA, Jun 2006.
    %
    %   Geir Evensen,
    %   Sequential data assimilation with a nonlinear quasi-geostrophic model using Monte Carlo methods to forecast error statistics,
    %   Journal of Geophysical Research: Oceans Vol. 99 C5, 1994, pp. 10143-10162.
    
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
        function obj = EnKF(name)
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
            %      Default name: 'EnKF'.
            %
            % Returns:
            %   << obj (EnKF)
            %      A new EnKF instance.
            
            if nargin < 1
                name = 'EnKF';
            end
            
            % Call superclass constructor
            obj = obj@ParticleFilter(name);
            
            obj.ensemble     = [];
            obj.ensembleSize = 1000;
        end
        
        function setEnsembleSize(obj, ensembleSize)
            % Set the size of the ensemble (i.e., the number of samples).
            % 
            % By default, 1000 ensemble members will be used.
            %
            % Parameters:
            %   >> ensembleSize (Positive scalar)
            %      The number of ensemble members used by the filter.
            
            if ~Checks.isPosScalar(ensembleSize)
                obj.error('InvalidEnsembleSize', ...
                          'ensembleSize must be a positive scalar.');
            end
            
            ensembleSize = ceil(ensembleSize);
            
            if isempty(obj.ensemble)
                obj.ensembleSize = ensembleSize;
            else
                obj.resample(ensembleSize);
            end
        end
        
        function ensembleSize = getEnsembleSize(obj)
            % Get the ensemble size of the filter.
            %
            % Returns:
            %   << ensembleSize (Positive scalar)
            %      The ensemble size used by he filter.
            
            ensembleSize = obj.ensembleSize;
        end
        
        function state = getState(obj)
            state = DiracMixture(obj.ensemble);
        end
        
        function [stateMean, stateCov] = getStateMeanAndCov(obj)
            if nargout == 1
                stateMean = Utils.getMeanAndCov(obj.ensemble);
            else
                [stateMean, stateCov] = Utils.getMeanAndCov(obj.ensemble);
            end
        end
    end
    
    methods (Access = 'protected')
        function performSetState(obj, state)
            obj.ensemble = state.drawRndSamples(obj.ensembleSize);
        end
        
        function performPrediction(obj, sysModel)
            if Checks.isClass(sysModel, 'SystemModel')
                obj.ensemble = obj.predictParticlesArbitraryNoise(sysModel, obj.ensemble, obj.ensembleSize);
            elseif Checks.isClass(sysModel, 'AdditiveNoiseSystemModel')
                obj.ensemble = obj.predictParticlesAdditiveNoise(sysModel, obj.ensemble, obj.ensembleSize);
            elseif Checks.isClass(sysModel, 'MixedNoiseSystemModel')
                obj.ensemble = obj.predictParticlesMixedNoise(sysModel, obj.ensemble, obj.ensembleSize);
            else
                obj.errorSysModel('SystemModel', ...
                                  'AdditiveNoiseSystemModel', ...
                                  'MixedNoiseSystemModel');
            end
        end
        
        function performUpdate(obj, measModel, measurement)
            obj.checkMeasurementVector(measurement);
            
            if Checks.isClass(measModel, 'MeasurementModel')
                obj.updateArbitraryNoise(measModel, measurement);
            elseif Checks.isClass(measModel, 'AdditiveNoiseMeasurementModel')
                obj.updateAdditiveNoise(measModel, measurement);
            elseif Checks.isClass(measModel, 'MixedNoiseMeasurementModel')
                obj.updateMixedNoise(measModel, measurement);
            else
                obj.errorMeasModel('MeasurementModel', ...
                                   'AdditiveNoiseMeasurementModel', ...
                                   'MixedNoiseMeasurementModel');
            end
        end
        
        function updateArbitraryNoise(obj, measModel, measurement)
            dimMeas  = size(measurement, 1);
            
            % Sample measurement noise
            noiseSamples = measModel.noise.drawRndSamples(obj.ensembleSize);
            
            % Propagate ensemble and noise through measurement equation
            measSamples = measModel.measurementEquation(obj.ensemble, noiseSamples);
            
            % Check computed measurements
            obj.checkComputedMeasurements(measSamples, dimMeas, obj.ensembleSize);
            
            obj.updateEnsemble(measurement, measSamples);
        end
        
        function updateAdditiveNoise(obj, measModel, measurement)
            dimNoise = measModel.noise.getDim();
            dimMeas  = size(measurement, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimNoise);
            
            % Propagate ensemble through measurement equation
            meas = measModel.measurementEquation(obj.ensemble);
            
            % Check computed measurements
            obj.checkComputedMeasurements(meas, dimMeas, obj.ensembleSize);
            
            % Sample additive measurement noise
            addNoiseSamples = measModel.noise.drawRndSamples(obj.ensembleSize);
            
            measSamples = meas + addNoiseSamples;
            
            obj.updateEnsemble(measurement, measSamples);
        end
        
        function updateMixedNoise(obj, measModel, measurement)
            dimAddNoise = measModel.additiveNoise.getDim();
            dimMeas     = size(measurement, 1);
            
            obj.checkAdditiveMeasNoise(dimMeas, dimAddNoise);
            
            % Sample measurement noise
            noiseSamples = measModel.noise.drawRndSamples(obj.ensembleSize);
            
            % Propagate ensemble and noise through measurement equation
            meas = measModel.measurementEquation(obj.ensemble, noiseSamples);
            
            % Check computed measurements
            obj.checkComputedMeasurements(meas, dimMeas, obj.ensembleSize);
            
            % Sample additive measurement noise
            addNoiseSamples = measModel.additiveNoise.drawRndSamples(obj.ensembleSize);
            
            measSamples = meas + addNoiseSamples;
            
            obj.updateEnsemble(measurement, measSamples);
        end
        
        function updateEnsemble(obj, measurement, measSamples)
            ensembleMean = sum(obj.ensemble, 2) / obj.ensembleSize;
            
            [~, measCov, ensembleMeasCrossCov] = Utils.getMeanCovAndCrossCov(ensembleMean, ...
                                                                             obj.ensemble, ...
                                                                             measSamples);
            
            [~, isNonPosDef] = chol(measCov);
            
            if isNonPosDef
                obj.ignoreMeas('Measurement covariance matrix is not positive definite.');
            end
            
            % Compute Kalman gain
            kalmanGain = ensembleMeasCrossCov / measCov;
            
            % Compute memberwise innovation
            innovation = bsxfun(@minus, measurement, measSamples);
            
            % Update ensemble
            obj.ensemble = obj.ensemble + kalmanGain * innovation;
        end
        
        function resample(obj, ensembleSize)
            idx = randi(obj.ensembleSize, 1, ensembleSize);
            
            obj.ensembleSize = ensembleSize;
            obj.ensemble     = obj.ensemble(:, idx);
        end
    end
    
    properties (Access = 'private')
        % The actual ensemble (set of samples)
        ensemble;
        
        % The number of ensemble members
        ensembleSize;
    end
end
