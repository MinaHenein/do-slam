classdef EnvironmentYASH < handle & matlab.mixin.Copyable
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess=protected,SetAccess=protected)
        EnvironmentPrimitives
        EnvironmentPoints
    end
    
    methods
        function obj = Environment(varargin)
            obj.EnvironmentPrimitives = EnvironmentPrimtive.empty(0,0);
            obj.EnvironmentPoints = EnvironmentPoints.empty(0,0);
        end
        
        function obj = addEnvironmentPrimitive(obj,EnvironmentPrimitive)
            EnvironmentPrimitive.set('ID',(numel(EnvironmentPrimitive)+1));
            obj.EnvironmentPrimitives(end+1) = EnvironmentPrimitive;
            
        end
        
        function obj = addEnvironmentPoint(obj,EnvironmentPoint)
            obj.EnvironmentPoints(end+1) = EnvironmentPoint;
        end
        
        figure = plot(obj,time);
    end
    
end

