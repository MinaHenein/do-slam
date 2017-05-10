classdef Object < ArrayGetSet & matlab.mixin.Heterogeneous
    %BaseObject is a hetergeneous superclass for Object and Geometry classes
    %   This class allows Object and Geometry class & subclass instances to 
    %   be stored in a heterogeneous array.
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        index
        pointIndexes
        vertexIndex
    end
    
    %% 2. Methods
    methods
    end
    
end

