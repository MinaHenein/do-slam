classdef EnvironmentALT < handle & matlab.mixin.Copyable
    %ENVIRONMENTALT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        environmentPrimitives
        environmentPoints
    end
    
    %% 2. Methods
    methods(Access = public)
        % Constructor
        function self = EnvironmentALT()
        end
        
        % add Rectangle 
        function self = addRectangle(self,sideLengths,nPoints,distribution,rectangleTrajectory)
            nSteps = numel(rectangleTrajectory.get('t'));
            
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
            
            %generate GP_Points *PUT IN PRIVATE METHOD
            rectanglePointTrajectories(nPoints) = PointTrajectory();
            for i = 1:nPoints
                iRelativePosition = rectanglePositionsRelative(:,i);
                iRelativePoints(nSteps)    = GP_Point();
                iRelativePoints.set('R3Position',repmat(iRelativePosition,1,nSteps),1:nSteps);
                iAbsolutePoints = iRelativePoints.RelativeToAbsolutePoint(rectangleTrajectory.get('poses'));
                rectanglePointTrajectories(i).set('t',rectangleTrajectory.get('t'));
                rectanglePointTrajectories(i).set('points',iAbsolutePoints);
            end
            
            % generate environment points
            rectanglePointIndexes = numel(self.EnvironmentPoints)+1:numel(self.EnvironmentPoints)+nPoints;
            EnvironmentPoints(rectanglePointIndexes) = EnvironmentPoint();
            for i = 1:nPoints
                EnvironmentPoints(rectanglePointIndexes(i)) = EnvironmentPoint(rectanglePointTrajectories(i),rectanglePointIndexes(i));
            end
            
            % initialise rectangle primitive
            rectangle = EP_Rectangle(sideLengths,rectangleTrajectory);
            %rectangle.set('index',numel(self.environmentPrimitives)+1);
            %rectangle.set('environmentPointIndexes',rectanglePointIndexes);
            
            % store in EnvironmentPrimitives, EnvironmentPoints
%             environmentPrimitives(end+1) = rectangle;
            
        end
    end
    
end

