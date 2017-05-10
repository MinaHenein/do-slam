classdef SensorEnvironment < ArrayGetSet
    %SENSORENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        points
        objects
    end
    
    properties(Dependent)
        nPoints
        nObjects
    end
    
    %% 2. Methods   
    % Dependent properties
    methods
        function nPoints = get.nPoints(self)
            nPoints = numel(self.points);
        end
        function nObjects = get.nObjects(self)
            nObjects = numel(self.objects);
        end
    end
    
    % Constructor
    methods(Access = public)
        function self = SensorEnvironment(environment)
            points(environment.nEnvironmentPoints) = Point();
            %loop over environmentPoints, create Points
            for i = 1:environment.nEnvironmentPoints
                points(i) = Point(environment.get('environmentPoints',i));
            end
            
            %loop over environmentPrimitives, create objects
            objects(environment.nEnvironmentPrimitives) = Object();
            for i = 1:environment.nEnvironmentPrimitives
                switch class(environment.get('environmentPrimitives',i))
                    case 'EP_Rectangle'
                        assert(logical(environment.get('environmentPrimitives',i).get('static')),'Error: plane must be formed from static rectangle')
                        objects(i) = GEO_Plane(environment.get('environmentPrimitives',i));
                    otherwise
                        error('Error: object conversion for %s not yet implemented',class(environment.get('environmentPrimitives',i)))
                end
            end
            
            %add
            self.points  = points;
            self.objects = objects;
        end
    end
    
    % Getter & Setter
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
            switch property
                case 'points'
                    if numel(varargin)==1
                        out = self.points(varargin{1});
                    else
                        out = self.points;
                    end
                case 'objects'
                    if numel(varargin)==1
                        out = self.objects(varargin{1});
                    else
                        out = self.objects;
                    end
                otherwise
                    out = self.(property);
            end
        	
        end
        
    end 
    
    
end

