classdef EnvironmentALT < handle & matlab.mixin.Copyable
    %ENVIRONMENTALT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        EnvironmentPrimitives
        EnvironmentPoints
    end
    
    %% 2. Methods
    methods(Access = public)
        % add Rectangle 
        function self = addRectangle(self,sideLengths,nPoints,distribution,rectangleTrajectory)
            % generate points 
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
            
            %
            % initialise rectangle primitive
            rectangle = EP_Rectangle(sideLengths,rectangleTrajectory);
        end
    end
    
end

