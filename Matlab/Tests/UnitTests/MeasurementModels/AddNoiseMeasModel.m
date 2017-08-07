
classdef AddNoiseMeasModel < AdditiveNoiseMeasurementModel
    % Example implementation of an AdditiveNoiseMeasurementModel.
    
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
        function obj = AddNoiseMeasModel(stateDecomp)
            if nargin < 1
                stateDecomp = false;
            end
            
            if stateDecomp
                obj.measMatrix = [ 3
                                  -0.5
                                   3  ];
            else
                obj.measMatrix = [3 -4
                                  0  2
                                  3  0];
            end
        end
        
        function measurements = measurementEquation(obj, stateSamples)
            measurements = obj.measMatrix * stateSamples;
        end
    end
    
    properties (SetAccess = 'private', GetAccess = 'public')
        measMatrix;
    end
end
