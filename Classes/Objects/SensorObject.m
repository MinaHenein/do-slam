%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 29/06/17
%--------------------------------------------------------------------------

classdef SensorObject < ArrayGetSet & matlab.mixin.Heterogeneous
    %BaseObject is a hetergeneous superclass for Object and Geometry classes
    %   This class allows Object and Geometry class & subclass instances to 
    %   be stored in a heterogeneous array.
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        index
        pointIndexes
        vertexIndex
        static
    end
    
    %% 2. Methods
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
            switch property
                case 'static'
                    out = self.trajectory.get(property);
                otherwise
                    out = self.(property);
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
    end
    
end

