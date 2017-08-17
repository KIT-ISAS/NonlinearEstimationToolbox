
classdef SampleCache < handle & matlab.mixin.Copyable
    % Abstract base class for a file system-based sample cache.
    %
    % SampleCache Methods:
    %   copy            - Copy a SampleCache instance.
    %   getSamples      - Retrieve samples from the sample cache.
    %   generateSamples - Generate and store samples in the sample cache.
    
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
        function obj = SampleCache(sampleCachePath)
            % Class constructor.
            %
            % Parameters:
            %   >> sampleCachePath (char)
            %      Path of the sample cache.
            %
            % Returns:
            %   << obj (SampleCache)
            %      A new SampleCache instance.
            
            if ~ischar(sampleCachePath)
                error('SampleCache:InvalidSampleCachePath', ...
                      'sampleCachePath must be a char.');
            end
            
            obj.data            = {};
            obj.sampleCachePath = sampleCachePath;
            
            if exist(obj.sampleCachePath, 'dir') == 0
                % Sample cache directory does not exist => Create it.
                if ~mkdir(obj.sampleCachePath)
                   error('SampleCache:CreateSampleCacheDirFailed', ...
                         'Could not create sample cache directory\n%s.', ...
                         obj.sampleCachePath);
                end
            end
        end
        
        function [samples, weights] = getSamples(obj, dimension, numSamples)
            % Retrieve samples from the sample cache.
            %
            % If the requested combination of dimension and number of samples is
            % currently not in the sample cache, the samples will be computed and
            % persistently stored in the file system.
            %
            % Parameters:
            %   >> dimension (Positive scalar)
            %      Dimension of the requested distribution.
            %
            %   >> numSamples (Positive scalar)
            %      Desired number of samples.
            %
            % Returns:
            %   << samples (Matrix of dimension dimension x numSamples)
            %      Set of columnwise-arranged samples.
            %
            %   << weights (Row vector with length numSamples)
            %      Set of corresponding columnwise-arranged sample weights.
            
            [dimension, numSamples] = obj.checkRequest(dimension, numSamples);
            
            if dimension > length(obj.data) || ...
               numSamples > length(obj.data{dimension}) || ...
               isempty(obj.data{dimension}{numSamples})
                % Samples not in memory => Load it!
                if ~obj.loadSamplesFromFile(dimension, numSamples)
                   % This dimension and number of sample combination
                   % doesn't exists in sample cache => Try to create and
                   % subsequently load it.
                   warning('SampleCache:SamplesNotExist', ...
                           ['The requested %d %dD samples are not in Sample Cache.\n' ...
                            'They will be computed for you. Please wait.'], ...
                           numSamples, dimension);
                   
                   obj.generateSampleFile(dimension, numSamples);
                   
                   obj.loadSamplesFromFile(dimension, numSamples);
                end
            end
            
            samples = obj.data{dimension}{numSamples}.samples;
            
            if nargout == 2
                weights = obj.data{dimension}{numSamples}.weights;
            end
        end
        
        function generateSamples(obj, dimension, numSamples, forceOverwrite)
            % Generate and store samples in the sample cache.
            %
            % Parameters:
            %   >> dimension (Positive scalar)
            %      Dimension of the distribution.
            %
            %   >> numSamples (Positive scalar)
            %      Desired number of samples.
            %
            %   >> forceOverwrite (Logical scalar)
            %      If true and there exists already an entry in the sample cache,
            %      it will be overwritten by a new set of samples. Otherwise, no
            %      samples will be computed.
            
            if nargin < 4
                forceOverwrite = false;
            end
            
            if ~Checks.isFlag(forceOverwrite)
                error('SampleCache:InvalidForceOverwriteFlag', ...
                      'forceOverwrite must be a logical scalar.');
            end
            
            [dimension, numSamples] = obj.checkRequest(dimension, numSamples);
            
            filename = obj.getSampleFilename(dimension, numSamples);
            
            if exist(filename, 'file') == 2 && ~forceOverwrite
                % Samples already exists and no explicit overwrite is
                % forced => Quit
                warning('SampleCache:SamplesAlreadyExists', ...
                        ['The requested %d %dD samples already exist in Sample Cache.\n' ...
                         'Use forceOverwrite to compute a new approximation.'], ...
                        numSamples, dimension);
                return;
            end
            
            obj.generateSampleFile(dimension, numSamples);
        end
    end
    
    methods (Abstract, Access = 'protected')
        checkDimAndNumSamplesCombination(obj, dimension, numSamples);
        
        [samples, weights] = computeSamples(obj, dimension, numSamples);
    end
    
    methods (Access = 'private')
        function [dimension, numSamples] = checkRequest(obj, dimension, numSamples)
            if ~Checks.isPosScalar(dimension)
                error('SampleCache:InvalidDimension', ...
                      'dimension must be a positive scalar.');
            end
            
            dimension = ceil(dimension);
            
            if ~Checks.isPosScalar(numSamples)
                error('SampleCache:InvalidNumberOfSamples', ...
                      'numSamples must be a positive scalar.');
            end
            
            numSamples = ceil(numSamples);
            
            obj.checkDimAndNumSamplesCombination(dimension, numSamples);
        end
        
        function success = loadSamplesFromFile(obj, dimension, numSamples)
            filename = obj.getSampleFilename(dimension, numSamples);
            
            if exist(filename, 'file') ~= 2
                % This dimension and number of samples combination
                % does not exist in sample csache.
                success = false;
                return;
            end
            
            fd = fopen(filename, 'r', 'l');
            
            if fd == -1
               error('SampleCache:OpenSampleFileFailed', ...
                     'Could not open sample file "%s".', filename);
            end
            
            % Read samples
            samples = fread(fd, [dimension, numSamples], 'double', 0, 'l');
            
            % Read weights
            weights = fread(fd, [1, numSamples], 'double', 0, 'l');
            
            fclose(fd);
            
            % Check read data
            if any(size(samples) ~= [dimension, numSamples])
               error('SampleCache:LoadSamplesFromFileFailed', ...
                     'Load samples from file "%s" failed: Invalid samples data.', filename);
            end
            
            if any(size(weights) ~= [1, numSamples])
               error('SampleCache:LoadSamplesFromFileFailed', ...
                     'Load samples from file "%s" failed: Invalid weights data.', filename);
            end
            
            obj.data{dimension}{numSamples}.samples = samples;
            obj.data{dimension}{numSamples}.weights = weights;
            
            success = true;
        end
        
        function generateSampleFile(obj, dimension, numSamples)
            % Compute samples
            [samples, weights] = obj.computeSamples(dimension, numSamples);
            
            % Save samples
            filename = obj.getSampleFilename(dimension, numSamples);
            
            % Store samples and weights in Little-Endian and column order
            fd = fopen(filename, 'w+', 'l');
            
            if fd == -1
                error('SampleCache:OpenSampleFileFailed', ...
                      'Could not open sample file "%s".', filename);
            end
            
            numElementsWritten = fwrite(fd, samples, 'double', 0, 'l');
            
            if numElementsWritten ~= (dimension * numSamples)
                fclose(fd);
                
                error('SampleCache:WritToFileFailed', ...
                      'Writing samples to file "%s" failed.', filename);
            end
            
            numElementsWritten = fwrite(fd, weights, 'double', 0, 'l');
            
            if numElementsWritten ~= numSamples
                fclose(fd);
                
                error('SampleCache:WriteToFileFailed', ...
                      'Writing weights to file "%s" failed.', filename);
            end
            
            fclose(fd);
        end
        
        function filename = getSampleFilename(obj, dimension, numSamples)
            filename = sprintf('%s/%dD-%dS.samples', ...
                               obj.sampleCachePath, dimension, numSamples);
        end
    end
    
    properties (Access = 'private')
        data;
        sampleCachePath;
    end
end
