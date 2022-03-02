%STARTUPSAV   Startup file
%   Change the name of this file to STARTUP.M. The file 
%   is executed when MATLAB starts up, if it exists 
%   anywhere on the path.  In this example, the
%   MAT-file generated during quitting using FINISHSAV
%   is loaded into MATLAB during startup.

%   Copyright 1984-2000 The MathWorks, Inc. 

%load matlab.mat

% Remove stupid MATLAB dir
cd('C:\Users\halva\OneDrive - Universitetet i Stavanger\Documents\')
!rmdir MATLAB

cd('C:\Users\halva\OneDrive - Universitetet i Stavanger\Documents\UNI\22_V\ELE130\Legoprosjekt\Lego\Matlab');

addpath('C:\Users\halva\OneDrive - Universitetet i Stavanger\Documents\UNI\22_V\ELE130\Legoprosjekt\Lego\Matlab\Joystick\PC');
addpath('C:\Users\halva\OneDrive - Universitetet i Stavanger\Documents\UNI\22_V\ELE130\Legoprosjekt\Lego\Matlab\MineFunksjoner');