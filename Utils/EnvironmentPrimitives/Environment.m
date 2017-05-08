classdef Environment < handle & matlab.mixin.Copyable
    %ENVIRONMENTALT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        environmentPrimitives
        environmentPoints
    end
    
    properties(Dependent)
        nEnvironmentPrimitives
        nEnvironmentPoints
    end
    
    %% 2. Methods
    % Dependent properties
    methods
        function nEnvironmentPrimitives = get.nEnvironmentPrimitives(self)
            nEnvironmentPrimitives = numel(self.environmentPrimitives);
        end
        
        function nEnvironmentPoints = get.nEnvironmentPoints(self)
            nEnvironmentPoints = numel(self.environmentPoints);
        end
    end
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = get(self,property,varargin)
            switch nargin
                case 2
                    out = [self.(property)];
                case 3
                    if strcmp('environmentPrimitives',property)
                        out(numel(varargin{1})) = EnvironmentPrimitiveALT();
                    elseif strcmp('environmentPoints',property)
                        out(numel(varargin{1})) = EnvironmentPoint();
                    else
                        error('Only provide location for environmentPrimitives/environmentPoints properties')
                    end
                    
                    for i = 1:numel(varargin{1})
                        out(i) = [self.(property)(varargin{1}(i))];
                    end
            end
        end
        
        function self = set(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Constructor
    methods(Access = public)
        function self = Environment()
           
        end
    end
    
    % Add Primitives & Points
    methods(Access = public)
        % Rectangle
        function self = addRectangle(self,sideLengths,nPoints,distribution,rectangleTrajectory)
            
            %relative positions
            switch distribution
                case 'uniform'
                    rectanglePositionsRelative = generateRectanglePoints(sideLengths,nPoints,'uniform');
                case 'edges'
                    rectanglePositionsRelative = generateRectanglePoints(sideLengths,nPoints,'edges');
                case 'mixed'
                    nCentrePoints = 0.7*nPoints;
                    nEdgePoints   = nPoints - nCentrePoints;
                    rectanglePositionsRelative = [generateRectanglePoints(sideLengths,nCentrePoints,'uniform'),...
                                                  generateRectanglePoints(sideLengths,nEdgePoints,'edges')];
            end
            
            % initialise rectangle primitive
            rectangle = EP_Rectangle(sideLengths,rectangleTrajectory);
            
            % add
            self.addPrimitiveAndPoints(rectangle,rectanglePositionsRelative);           
            
        end
        
        % Primitive from relativePositions
        function self = addPrimitive(self,positionsRelative,trajectory)
            primitive = EP_Default();
            primitive.set('trajectory',trajectory);
            self.addPrimitiveAndPoints(primitive,positionsRelative);
        end
        
    end
       
    % Add 
    methods(Access = private)
        % from environment primitive and positions of points in relative
        % coords wrt to environment primitive, construct environment points
        % and do indexing
        function self = addPrimitiveAndPoints(self,envPrimitive,relativePositions)
            nPoints = size(relativePositions,2);
            nSteps = numel(envPrimitive.get('trajectory').get('t'));

            %compute absolute positions of rectangle points from relative
            %positions and rectangle trajectory
            pointTrajectories(nPoints) = PointTrajectory();
            for i = 1:nPoints
                iRelativePosition = relativePositions(:,i);
                iRelativePoints(nSteps) = GP_Point();
                iRelativePoints.set('R3Position',repmat(iRelativePosition,1,nSteps),1:nSteps);
                iAbsolutePoints = iRelativePoints.RelativeToAbsolutePoint(envPrimitive.get('trajectory').get('poses'));
                pointTrajectories(i).set('t',envPrimitive.get('trajectory').get('t'));
                pointTrajectories(i).set('points',iAbsolutePoints);
            end
            
            % generate environment points
            pointIndexes = self.nEnvironmentPoints+1:self.nEnvironmentPoints+nPoints;
            envPoints(nPoints) = EnvironmentPoint();
            for i = 1:nPoints
                envPoints(i) = EnvironmentPoint(pointTrajectories(i),pointIndexes(i));
            end
            
            envPrimitive.set('index',numel(self.environmentPrimitives)+1);
            envPrimitive.set('environmentPointIndexes',pointIndexes);
            
            % store in EnvironmentPrimitives, EnvironmentPoints
            self.environmentPrimitives = [self.environmentPrimitives envPrimitive];
            self.environmentPoints     = [self.environmentPoints envPoints];
        end
    end
    
    % Plot
    methods(Access = public)
        function h = plot(self,timeStep)
            %get environment points positions at timeStep
            positions = self.get('environmentPoints'). ...
                                    get('trajectory'). ...
                                    get('points','timeStep',timeStep). ...
                                    get('R3Position');
            h = plot3(positions(1,:),positions(2,:),positions(3,:),'k.');
            
            %*TODO: loop over primitives and plot too
        end
    end
end

