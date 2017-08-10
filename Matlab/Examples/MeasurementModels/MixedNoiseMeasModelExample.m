
classdef MixedNoiseMeasModelExample < MixedNoiseMeasurementModel
    methods
        function obj = MixedNoiseMeasModelExample()
            % Specify the Gaussian system noise.
            % Of course, you do not have to call setNoise() and setAdditiveNoise() in the constructor.
            % You can also set/change the noise after creating a MixedNoiseMeasModelExample object.
            obj.setNoise(Gaussian(0, 0.1));
            obj.setAdditiveNoise(Gaussian(zeros(2, 1), eye(2)));
        end
        
        function measurements = measurementEquation(obj, stateSamples, noiseSamples)
            p = stateSamples(1, :);
            q = stateSamples(2, :);
            v = noiseSamples;
            
            measurements = [p .* q.^2 .* v.^2
                            p.^2 + 3*q .* v.^3];
        end
        
        function [stateJacobian, noiseJacobian, ...
                  stateHessians, noiseHessians] = derivative(obj, nominalState, nominalNoise)
            p = nominalState(1);
            q = nominalState(2);
            v = nominalNoise;
            
            % Jacobians
            stateJacobian = [q^2*v^2 2*p*q*v^2
                               2*p     3*v^3  ];
            
            noiseJacobian = [2*p*q^2*v
                              9*q*v^2 ];
            
            % Hessians
            if nargout >= 3
                % Hessian for p_k
                stateHessians(:, :, 1) = [   0    2*q*v^2
                                          2*q*v^2 2*p*v^2];
                
                % Hessian for q_k
                stateHessians(:, :, 2) = [2 0
                                          0 0];
                
                % Hessian for p_k
                noiseHessians(:, :, 1) = 2*p*q^2;
                
                % Hessian for q_k
                noiseHessians(:, :, 2) = 18*q*v;
            end
        end
    end
end
