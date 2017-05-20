classdef SimulatedEnvironmentSensor < Sensor
    %SimulatedEnvironmentSensor represents a sensor used to generate
    %measurements of a primitives and points in the Environment class
    %   -SimulatedEnvironmentSensor converts the environment primitives and
    %    environment points to objects and points
    %   -This intermediate object representation allows more freedom in the
    %    kinds of observations and constraints which can be generated from
    %    environment primitives and points
    %   -ie different SimulatedEnvironmentSensor subclasses can rerepresent
    %    the same Environment class in different ways (ie planes vs
    %    rectangles)
    %   -Measurements of these points are generated and stored in a graph
    %    file, along with a ground truth graph file
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        points
        objects
        fieldOfView
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
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Pose','R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    out = self.trajectory.get(property,varargin{1});
                case 'static'
                    out = self.trajectory.get(property);
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
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Constructor
    methods(Access = public)
        function self = SimulatedEnvironmentSensor()
        end
    end
    
    % Add environment
    methods(Access = public)
        function self = addEnvironment(self,environment)
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
    
    % Add camera
    methods(Access = public)
        function self = addCamera(self,fieldOfView,trajectory)
            self.fieldOfView = fieldOfView;
            self.trajectory  = trajectory;
        end
    end
    
    %Declare external methods
    methods(Access = public)
        % point visibility
        [visibility,relativePoint] = pointVisible(self,point,t)
        % Measurements
        generateMeasurements(self,config)
        
    end
    
end
