
function varargout = GLCD(varargin)
    % Wrapper function to call the platform-specific GLCD MEX files.
    %
    % Execute GLCD without parameters to get more information.
    
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
    
    varargout = cell(1, nargout);
    
    switch computer()
        case 'GLNXA64'
            if verLessThan('matlab', '8.2')
                error('GLCD:UnsupportedMatlabVersion', ...
                      'At least Matlab R2013b is required.');
            else
                [varargout{:}] = GLCD_R2013b(varargin{:});
            end
        case 'PCWIN64'
            if verLessThan('matlab', '8.3')
                error('GLCD:UnsupportedMatlabVersion', ...
                      'At least Matlab R2014a is required.');
            else
                [varargout{:}] = GLCD_R2014a(varargin{:});
            end
        otherwise
            error('GLCD:UnsupportedPlatform', ...
                  'Only Windows 64-bit and Linux are supported.');
    end
end
