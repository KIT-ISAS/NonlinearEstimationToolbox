
function compileMexOpenMP(varargin)
    % Compile a MEX binary with OpenMP support enabled
    
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
    
    % Enable OpenMP
    if ispc()
        % On Windows
        options = { 'OPTIMFLAGS=$OPTIMFLAGS /openmp' };
    elseif ismac()
        % On Mac OS X, OpenMP does not seem to be available in clang/Xcode
        % (at least for some versions). Hence, we do not enable it by default.
        
        warning('OpenMP is disabled by default on Mac OS X! You can enable it by editing Mex/compileMexOpenMP.m.');
        
        options = { };
        
        % To enable OpenMP, uncomment the following options:
%         options = { 'CXXFLAGS=$CXXFLAGS -fopenmp', ...
%                     'LDFLAGS=$LDFLAGS -fopenmp' };
    elseif isunix()
        % On Linux
        options = { 'CXXFLAGS=$CXXFLAGS -fopenmp', ...
                    'LDFLAGS=$LDFLAGS -fopenmp' };
    else
        error('Unsupported platform.');
    end
    
    compileMex(varargin{:}, options{:});
end
