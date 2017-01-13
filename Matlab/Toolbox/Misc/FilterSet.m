
classdef FilterSet < handle
    % A set of filters.
    %
    % FilterSet Methods:
    %   FilterSet         - Class constructor.
    %   add               - Add a filter to the set.
    %   remove            - Remove a filter from the set
    %   get               - Get a particular filter from the set.
    %   getIndex          - The current index of a filter in the set.
    %   getNumFilters     - Get the current number of filters in the set.
    %   getNames          - Get the names of all filters in the set.
    %   setStates         - Set the system state for all filters in the set.
    %   getStates         - Get the current system state of all filters in the set.
    %   getStateDim       - Get the dimension of the current system state.
    %   predict           - Predict all filters in the set using the given system model.
    %   predictSingle     - Predict the filter with the given id and system model.
    %   update            - Update all filters in the set using the given measurement model and measurements.
    %   updateSingle      - Update the filter with the given id, measurement model, and measurements.
    %   step              - Predict and update all filters in the set using the given system model, measurement model, and measurements.
    %   stepSingle        - Predict and update the filter with the given id, system model, measurement model, and measurements.
    %   getPointEstimates - Get the point estimates of all filters in the set.
    
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
        function obj = FilterSet()
            % Default constructor.
            %
            % Returns:
            %   << obj (FilterSet)
            %      A new FilterSet instance.
            
            obj.filterMap = containers.Map('KeyType', 'char', ...
                                           'ValueType', 'any');
            
            obj.filters    = obj.filterMap.values();
            obj.numFilters = obj.filterMap.length();
            obj.dimState   = 0;
        end
        
        function add(obj, filter)
            % Add a filter to the set.
            %
            % Parameters:
            %   >> filter (Subclass of Filter)
            %      The filter to be added.
            
            if ~Checks.isClass(filter, 'Filter')
                error('FilterSet:InvalidFilter', ...
                      'filter must be a subclass of Filter.');
            end
            
            filterName = filter.getName();
            
            index = obj.getMapIndex(filterName);
            
            if ~isempty(index)
                error('FilterSet:InvalidFilterName', ...
                      'Filter name "%s" already exists.', filterName);
            end
            
            obj.filterMap(filterName) = filter;
            
            obj.filters    = obj.filterMap.values();
            obj.numFilters = obj.filterMap.length();
        end
         
        function remove(obj, filterName)
            % Remove a filter from the set.
            %
            % Parameters:
            %   >> name (Char)
            %      Name of the filter to be removed.
            
            if ~ischar(filterName)
                error('FilterSet:InvalidFilterName', ...
                      'filterName must be a char.');
            end
            
            index = obj.getMapIndex(filterName);
            
            if isempty(index)
                error('FilterSet:NoFilter', ...
                      'There exists no filter with the name "%s".', filterName);
            end
            
            obj.filterMap.remove(filterName);
            
            obj.filters    = obj.filterMap.values();
            obj.numFilters = obj.filterMap.length();
        end
        
        function filter = get(obj, id)
            % Get a particular filter from the set.
            %
            % Parameters:
            %   >> id (Char or scalar)
            %      The filter id can be either the filter name or its index in the set.
            %
            % Returns:
            %   << filter (Subclass of Filter)
            %      The filter corresponding to the given id.
            
            if ischar(id)
                try
                    filter = obj.filterMap(id);
                catch ex
                    if strcmp(ex.identifier, 'MATLAB:Containers:Map:NoKey')
                        error('FilterSet:NoFilter', ...
                              'There exists no filter with the name "%s".', id);
                    else
                        rethrow(ex);
                    end
                end
            elseif Checks.isScalar(id)
                try
                    filter = obj.filters{id};
                catch ex
                    if strcmp(ex.identifier, 'MATLAB:badsubscript')
                        error('FilterSet:NoFilter', ...
                              'Filter index out of range.');
                    else
                        rethrow(ex);
                    end
                end
            else
                error('FilterSet:InvalidIdentifier', ...
                      'id must be a filter name or an index.');
            end
        end
        
        function index = getIndex(obj, filterName)
            % The current index of a filter in the set.
            %
            % Note: The index of a filter may have changed when a filter was added or removed.
            %
            % Parameters:
            %   >> name (Char)
            %      Name of the filter to be removed.
            %
            % Returns:
            %   << index (Scalar)
            %      Current index of the filter in the set.
            
            if ischar(filterName)
                index = obj.getMapIndex(filterName);
                
                if isempty(index)
                    error('FilterSet:NoFilter', ...
                          'There exists no filter with the name "%s".', filterName);
                end
            else
                error('FilterSet:InvalidFilterName', ...
                      'filterName must be a char.');
            end
        end
        
        function numFilters = getNumFilters(obj)
            % Get the current number of filters in the set.
            % 
            % Returns:
            %   << numFilters (Scalar)
            %      The current number of filters in the set.
            
            numFilters = obj.numFilters;
        end
        
        function names = getNames(obj)
            % Get the names of all filters in the set.
            %
            % Returns:
            %   << names (Cell array of chars)
            %      Each cell array element contains the name of a filter.
            
            names = obj.filterMap.keys();
        end
        
        function setStates(obj, state)
            % Set the system state for all filters in the set.
            %
            % Parameters:
            %   >> state (Subclass of Distribution)
            %      The new system state.
            
            obj.forAllFilters(@setState, state);
            
            obj.dimState = state.getDimension();
        end
        
        function states = getStates(obj)
            % Get the current system state of all filters in the set.
            %
            % Returns:
            %   << states (Cell array of Distributions)
            %      Each cell array element contains the current state of a filter.
            
            states = cell(1, obj.numFilters);
            
            for i = 1:obj.numFilters
                states{i} = obj.filters{i}.getState();
            end
        end
        
        function dim = getStateDim(obj)
            % Get the dimension of the current system state.
            %
            % Returns:
            %   << dim (Scalar)
            %      The dimension of the current system state.
            
            dim = obj.dimState;
        end
        
        function runtimes = predict(obj, sysModel)
            % Predict all filters in the set using the given system model.
            %
            % Parameters:
            %   >> sysModel (Arbitrary class (filter dependent))
            %      System model that provides the mapping between the prior system
            %      state and the predicted state (i.e., the system state's temporal evolution).
            %
            % Returns:
            %   << runtimes (Array)
            %      Contains the prediction runtimes for all filters.
            
            if nargout == 1
                runtimes = obj.forAllFiltersRuntime(@predict, sysModel);
            else
                obj.forAllFilters(@predict, sysModel);
            end
        end
        
        function runtime = predictSingle(obj, id, sysModel)
            % Predict the filter with the given id and system model.
            %
            % Parameters:
            %   >> id (Char or scalar)
            %      The filter id can be either the filter name or its index in the set.
            %
            %   >> sysModel (Arbitrary class (filter dependent))
            %      System model that provides the mapping between the prior system
            %      state and the predicted state (i.e., the system state's temporal evolution).
            %
            % Returns:
            %   << runtime (Scalar)
            %      Runtime of the prediction.
            
            if nargout == 1
                runtime = obj.executeFilterRuntime(id, @predict, sysModel);
            else
                obj.executeFilter(id, @predict, sysModel);
            end
        end
        
        function runtimes = update(obj, measModel, measurements)
            % Update all filters in the set using the given measurement model and measurements.
            %
            % Parameters:
            %   >> measModel (Arbitrary class (filter dependent))
            %      Measurement model that provides the mapping between a measurement and the system state.
            %
            %   >> measurements (Matrix)
            %      Column-wise arranged measurement vectors, where each column represents an individual
            %      measurement. In case of two or more measurements (i.e., two or more columns), the
            %      filter assumes that the measurements originate from the same measurement model and
            %      i.i.d. measurement noise. For example, in case of a measurement model h(x, v) and two
            %      measurements m1 and m2 the filter assumes
            %
            %          m1 = h(x, v) and m2 = h(x, v) .
            %
            %      The advantage is that one has to set the measurement noise v only for one
            %      measurement, no matter how many measurements will be provided in one filter step.
            %      That is, the measurement noise is assumed to be i.i.d. for all measurements.
            %      However, in case of non-i.i.d. measurement noise 
            %
            %          m1 = h(x, v1) and m2 = h(x, v2)
            %
            %      (e.g., existing correlations between noise for different measurements or in general
            %      different noise for different measurements) one has to explicitly stack the
            %      measurement noise to v = [v1; v2] and pass the measurements m1 and m2 as a stacked
            %      measurement vector m = [m1; m2].
            %
            % Returns:
            %   << runtimes (Array)
            %      Contains the update runtimes for all filters.
            
            if nargout == 1
                runtimes = obj.forAllFiltersRuntime(@update, measModel, measurements);
            else
                obj.forAllFilters(@update, measModel, measurements);
            end
        end
        
        function runtime = updateSingle(obj, id, measModel, measurements)
            % Update the filter with the given id, measurement model, and measurements.
            %
            % Parameters:
            %   >> id (Char or scalar)
            %      The filter id can be either the filter name or its index in the set.
            %
            %   >> measModel (Arbitrary class (filter dependent))
            %      Measurement model that provides the mapping between a measurement and the system state.
            %
            %   >> measurements (Matrix)
            %      Column-wise arranged measurement vectors, where each column represents an individual
            %      measurement. In case of two or more measurements (i.e., two or more columns), the
            %      filter assumes that the measurements originate from the same measurement model and
            %      i.i.d. measurement noise. For example, in case of a measurement model h(x, v) and two
            %      measurements m1 and m2 the filter assumes
            %
            %          m1 = h(x, v) and m2 = h(x, v) .
            %
            %      The advantage is that one has to set the measurement noise v only for one
            %      measurement, no matter how many measurements will be provided in one filter step.
            %      That is, the measurement noise is assumed to be i.i.d. for all measurements.
            %      However, in case of non-i.i.d. measurement noise
            %
            %          m1 = h(x, v1) and m2 = h(x, v2)
            %
            %      (e.g., existing correlations between noise for different measurements or in general
            %      different noise for different measurements) one has to explicitly stack the
            %      measurement noise to v = [v1; v2] and pass the measurements m1 and m2 as a stacked
            %      measurement vector m = [m1; m2].
            %
            % Returns:
            %   << runtime (Scalar)
            %      Runtime of the measurement update.
            
            if nargout == 1
                runtime = obj.executeFilterRuntime(id, @update, measModel, measurements);
            else
                obj.executeFilter(id, @update, measModel, measurements);
            end
        end
        
        function runtimes = step(obj, sysModel, measModel, measurements)
            % Predict and update all filters in the set using the given system model, measurement model, and measurements.
            %
            % Parameters:
            %   >> sysModel (Arbitrary class (filter dependent))
            %      System model that provides the mapping between the prior system
            %      state and the predicted state (i.e., the system state's temporal evolution).
            %
            %   >> measModel (Arbitrary class (filter dependent))
            %      Measurement model that provides the mapping between measurements and the system state.
            %
            %   >> measurements (Matrix)
            %      Column-wise arranged measurement vectors, where each column represents an individual
            %      measurement. In case of two or more measurements (i.e., two or more columns), the
            %      filter assumes that the measurements originate from the same measurement model and
            %      i.i.d. measurement noise. For example, in case of a measurement model h(x, v) and two
            %      measurements m1 and m2 the filter assumes
            %
            %          m1 = h(x, v) and m2 = h(x, v) .
            %
            %      The advantage is that one has to set the measurement noise v only for one
            %      measurement, no matter how many measurements will be provided in one filter step.
            %      That is, the measurement noise is assumed to be i.i.d. for all measurements.
            %      However, in case of non-i.i.d. measurement noise
            %
            %          m1 = h(x, v1) and m2 = h(x, v2)
            %
            %      (e.g., existing correlations between noise for different measurements or in general
            %      different noise for different measurements) one has to explicitly stack the
            %      measurement noise to v = [v1; v2] and pass the measurements m1 and m2 as a stacked
            %      measurement vector m = [m1; m2].
            %
            % Returns:
            %   << runtimes (Array)
            %      Contains the runtimes for all filters.
            
            if nargout == 1
                runtimes = obj.forAllFiltersRuntime(@step, sysModel, measModel, measurements);
            else
                obj.forAllFilters(@step, sysModel, measModel, measurements);
            end
        end
        
        function runtime = stepSingle(obj, id, sysModel, measModel, measurements)
            % Predict and update the filter with the given id, system model, measurement model, and measurements.
            %
            % Parameters:
            %   >> id (Char or scalar)
            %      The filter id can be either the filter name or its index in the set.
            %
            %   >> sysModel (Arbitrary class (filter dependent))
            %      System model that provides the mapping between the prior system
            %      state and the predicted state (i.e., the system state's temporal evolution).
            %
            %   >> measModel (Arbitrary class (filter dependent))
            %      Measurement model that provides the mapping between measurements and the system state.
            %
            %   >> measurements (Matrix)
            %      Column-wise arranged measurement vectors, where each column represents an individual
            %      measurement. In case of two or more measurements (i.e., two or more columns), the
            %      filter assumes that the measurements originate from the same measurement model and
            %      i.i.d. measurement noise. For example, in case of a measurement model h(x, v) and two
            %      measurements m1 and m2 the filter assumes
            %
            %          m1 = h(x, v) and m2 = h(x, v) .
            %
            %      The advantage is that one has to set the measurement noise v only for one
            %      measurement, no matter how many measurements will be provided in one filter step.
            %      That is, the measurement noise is assumed to be i.i.d. for all measurements.
            %      However, in case of non-i.i.d. measurement noise
            %
            %          m1 = h(x, v1) and m2 = h(x, v2)
            %
            %      (e.g., existing correlations between noise for different measurements or in general
            %      different noise for different measurements) one has to explicitly stack the
            %      measurement noise to v = [v1; v2] and pass the measurements m1 and m2 as a stacked
            %      measurement vector m = [m1; m2].
            %
            % Returns:
            %   << runtime (Scalar)
            %      Runtime of the combined prediction and update.
            
            if nargout == 1
                runtime = obj.executeFilterRuntime(id, @step, sysModel, measModel, measurements);
            else
                obj.executeFilter(id, @step, sysModel, measModel, measurements);
            end
        end
        
        function [pointEstimates, uncertainties] = getPointEstimates(obj)
            % Get the point estimates of all filters in the set.
            %
            % Returns:
            %   << pointEstimates (Matrix)
            %      Column-wise arranged point estimates of all filters.
            %
            %   << uncertainties (3D matrix of positive definite matrices)
            %      Uncertainties of the current point estimates  of all filters
            %      (e.g., covariance matrices) arranged along the 3rd dimension.
            
            if nargout == 1
                pointEstimates = nan(obj.dimState, obj.numFilters);
                
                for i = 1:obj.numFilters
                    pointEstimates(:, i) = obj.filters{i}.getPointEstimate();
                end
            else
                pointEstimates = nan(obj.dimState, obj.numFilters);
                uncertainties  = nan(obj.dimState, obj.dimState, obj.numFilters);
                
                for i = 1:obj.numFilters
                    [pointEstimates(:, i), uncertainties(:, :, i)] = obj.filters{i}.getPointEstimate();
                end
            end
        end
    end
    
    methods (Access = 'protected')
        function index = getMapIndex(obj, name)
            index = find(strcmp(obj.filterMap.keys(), name), 1);
        end
        
        function forAllFilters(obj, func, varargin)
            cellfun(@(f) func(f, varargin{:}), obj.filters);
        end
        
        function runtimes = forAllFiltersRuntime(obj, func, varargin)
            runtimes = cellfun(@(f) func(f, varargin{:}), obj.filters);
        end
        
        function executeFilter(obj, id, func, varargin)
            filter = obj.get(id);
            
            func(filter, varargin{:});
        end
        
        function runtimes = executeFilterRuntime(obj, id, func, varargin)
            filter = obj.get(id);
            
            runtimes = func(filter, varargin{:});
        end
    end
    
    properties (Access = 'protected')
        % Key (filter name) / value (filter itself) storage.
        filterMap;
        
        % Cell array of filters.
        filters;
        
        % Number of filters hold by the set.
        numFilters;
        
        % State dimension of all filters.
        dimState;
    end
end
