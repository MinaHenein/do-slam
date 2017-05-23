%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef Sensor < ArrayGetSet & matlab.mixin.Heterogeneous
    %SENSOR is a base class to represent sensors in the environment
    %   It inherits from matlab.mixin.Heterogeneous which allows different
    %   sensor subclasses to be stored together in object arrays
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
    end
    
    %% 2. Methods
    
end

