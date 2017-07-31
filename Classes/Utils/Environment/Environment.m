%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 16/06/17
%--------------------------------------------------------------------------

classdef Environment < ArrayGetSet
    %Environment class stores environment primitives and environment points
    %and has methods to construct and manipulate them
    
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
    
    % Constructor
    methods(Access = public)
        function self = Environment()
        end
    end
    
    % Get & Set
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
            switch property
                case 'environmentPoints'
                    if numel(varargin)==1
                        out = self.environmentPoints(varargin{1});
                    else
                        out = self.environmentPoints;
                    end
                case 'environmentPrimitives'
                    if numel(varargin)==1
                        out = self.environmentPrimitives(varargin{1});
                    else
                        out = self.environmentPrimitives;
                    end
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
        
    end
    
    % Construct primitives
    methods(Access = public)
        % Default - any primitive formed from relative positions of points
        % it contains
        function self = addStaticPoints(self,positionsAbsolute)
            nPoints = size(positionsAbsolute,2);
            
            %initialise points
            pointIndexes   = self.nEnvironmentPoints + 1:self.nEnvironmentPoints + nPoints;
            points(nPoints) = EnvironmentPoint(); % preallocate
            
            for i=1:nPoints
                trajectory = StaticPointTrajectory(positionsAbsolute(:,i));
                points(i) = EnvironmentPoint(pointIndexes(i),trajectory);
            end
            
            %add to self
            self.environmentPoints = [self.environmentPoints points];
        end
        
        function self = addPrimitive(self,positionsRelative,parameterisation,trajectory)
            nPoints = size(positionsRelative,2);
            
            %initialise points
            points(nPoints) = EnvironmentPoint();
            for i = 1:nPoints
                iRelativePoint = GP_Point(positionsRelative(:,i),parameterisation);
                % trajectory of each point is represented with position
                % relative to primitive trajectory
                points(i).set('trajectory',RelativePointTrajectory(trajectory,iRelativePoint));
            end
            
            %construct primitive
            primitive = EnvironmentPrimitive();
            primitive.set('trajectory',trajectory);
            
            %pair primitive and points
            self.addPrimitiveAndPoints(primitive,points);
        end
        
        % Rectangle
        function self = addRectangle(self,sideLengths,nPoints,distribution,rectangleTrajectory)
            
            %relative positions
            switch distribution
                case 'uniform' % distribute points randomly on rectangle
                    rectanglePositionsRelative = generateRectanglePoints(sideLengths,nPoints,'uniform');
                case 'edges'   % distribute points randomly on edges only
                    rectanglePositionsRelative = generateRectanglePoints(sideLengths,nPoints,'edges');
                case 'mixed'   % combine uniform and edge distribution
                    nCentrePoints = 0.7*nPoints;
                    nEdgePoints   = nPoints - nCentrePoints;
                    rectanglePositionsRelative = [generateRectanglePoints(sideLengths,nCentrePoints,'uniform'),...
                                                  generateRectanglePoints(sideLengths,nEdgePoints,'edges')];
            end
                        
            %initialise points
            rectanglePoints(nPoints) = EnvironmentPoint();
            for i = 1:nPoints
                iRelativePoint = GP_Point(rectanglePositionsRelative(:,i));
                % trajectory of each point is represented with position
                % relative to primitive trajectory
                rectanglePoints(i).set('trajectory',RelativePointTrajectory(rectangleTrajectory,iRelativePoint));
            end
            
            %initialise EP_Rectangle primitive
            rectanglePrimitive = EP_Rectangle(sideLengths,rectangleTrajectory);
            
            %pair primitive and points
            self.addPrimitiveAndPoints(rectanglePrimitive,rectanglePoints);
            
        end
       
        
        function self = addEllipsoid(self,radii,N,parameterisation,trajectory)
            % create relative points
            [positions, links] = generateEllipsoidPoints(radii,N);
            nPositions = size(positions,2);
            points = positions(:,randsample(nPositions,floor(nPositions/5)));
            nPoints = size(points,2);
            
            %initialise points
            ellipsoidPoints(nPoints) = EnvironmentPoint();
            for i=1:nPoints
                iRelativePoint = GP_Point(points(:,i),parameterisation);
                ellipsoidPoints(i).set('trajectory',RelativePointTrajectory(trajectory,iRelativePoint));
            end
            
            meshPoints = GP_Point.empty();
            for i=1:nPositions
                meshPoints(i) = GP_Point(positions(:,i));
            end
            
            %initialise EP_Default primitive
            ellipsoidPrimitive = EP_Default();
            ellipsoidPrimitive.set('trajectory',trajectory);
            ellipsoidPrimitive.set('meshPoints',meshPoints);
            ellipsoidPrimitive.set('meshLinks',links);
            
            %pair primitive and points
            self.addPrimitiveAndPoints(ellipsoidPrimitive,ellipsoidPoints);
        end
    end
    
    
    % Add primitive & points
    %   adds primitive and points to environment
    %   adds indexes to pair primitive and points together
    methods(Access = private, Hidden = true)
        function self = addPrimitiveAndPoints(self,primitive,points)
            primitiveIndex = self.nEnvironmentPrimitives + 1;
            pointIndexes   = self.nEnvironmentPoints + 1:self.nEnvironmentPoints + numel(points);
            
            %set primitive indexes
            primitive.set('index',primitiveIndex);
            primitive.set('pointIndexes',pointIndexes);
            
            %set point indexes
            points.set('index',pointIndexes);
            points.set('primitiveIndexes',repmat(primitiveIndex,1,numel(points)));
            
            %add to self
            self.environmentPrimitives = [self.environmentPrimitives primitive];
            self.environmentPoints = [self.environmentPoints points];
        end
    end

    % plot
    methods(Access = public)
        function plot(self,t)
            %identify dynamic points and dynamic primitives
            staticPointLogical      = self.get('environmentPoints').get('static');
            staticPrimitiveLogical  = self.get('environmentPrimitives').get('static');
            dynamicPointLogical     = ~staticPointLogical;
            dynamicPrimitiveLogical = ~staticPrimitiveLogical;
            staticPointIndexes      = find(staticPointLogical);
            staticPrimitiveIndexes  = find(staticPrimitiveLogical);
            dynamicPointIndexes     = find(dynamicPointLogical);
            dynamicPrimitiveIndexes = find(dynamicPrimitiveLogical);
            self.plotStatic(staticPointIndexes,staticPrimitiveIndexes)
            self.plotDynamic(dynamicPointIndexes,dynamicPrimitiveIndexes,t)
        end
    end
    
    methods(Access = private, Hidden = true)
        % plot static features in environment
        function plotStatic(self,staticPointIndexes,staticPrimitiveIndexes)
            nPoints     = numel(staticPointIndexes);
            nPrimitives = numel(staticPrimitiveIndexes);
            %get point positions
            positions = self.environmentPoints(staticPointIndexes).get('R3Position',0);
            %plot positions
            if ~isempty(positions)
                plot3(positions(1,:),positions(2,:),positions(3,:),'k.')
            end
            %plot primitives
%             for i = 1:staticPrimitiveIndexes
%                 self.environmentPrimitives(staticPrimitiveIndexes(i)).plot()
%             end
        end

        % plot dynamic features of environment over time steps t
        function plotDynamic(self,dynamicPointIndexes,dynamicPrimitiveIndexes,t)
            nSteps = numel(t);
            for i = 1:nSteps
                nPoints     = numel(dynamicPointIndexes);
                nPrimitives = numel(dynamicPrimitiveIndexes);
                %get point positions
                positions = self.environmentPoints(dynamicPointIndexes).get('R3Position',t(i));
                %plot positions
                if positions
                    h1 = plot3(positions(1,:),positions(2,:),positions(3,:),'k.');
                end
                %plot primitives
                h2 = {};
                for p = dynamicPrimitiveIndexes
                    if isa(self.get('environmentPrimitives',p),'EP_Default')
                        meshPoints = self.get('environmentPrimitives',p).get('meshPointsAbsolute',t(i)).get('R3Position');
                        meshLinks = self.get('environmentPrimitives',p).get('meshLinks');
                        h2{end+1} = trimesh(meshLinks,meshPoints(1,:)',meshPoints(2,:)',meshPoints(3,:)','edgecolor','y');                
                    end
                end

                %draw current timestep, delete
                drawnow
                pause(0)
                if i < nSteps
                    if (positions) 
                        delete(h1) 
                    end
                    for p=1:numel(h2)
                        delete(h2{p})
                    end
                end
            end
        end
    end
end

