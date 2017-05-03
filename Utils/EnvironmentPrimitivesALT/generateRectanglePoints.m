function [positions] = generateRectanglePoints(sideLengths,nPoints,distribution)
%GENERATERECTANGLEPOINTS generates points on rectangle
%   rectangle lies in XY plane
%   sideLengths = [xLength,yLength]
%   corners = (0,0), (xLength,0), (0,yLength), (xLength,yLength)
%   can distribute points on edges or evenly

positions = zeros(3,nPoints);
switch distribution
    case 'edges'
        xRatio = sideLengths(1)/sum(sideLengths);
        yRatio = 1 - xRatio;
        edge1Indexes = 1:ceil(nPoints*xRatio/2);
        edge2Indexes = edge1Indexes(end)+1:edge1Indexes(end)+floor(nPoints*xRatio/2);
        edge3Indexes = edge2Indexes(end)+1:edge2Indexes(end)+ceil(nPoints*yRatio/2);
        edge4Indexes = edge3Indexes(end)+1:nPoints;
        positions(1,edge1Indexes) = sideLengths(1)*rand(1,numel(edge1Indexes));
        positions(1,edge2Indexes) = sideLengths(1)*rand(1,numel(edge2Indexes));
        positions(2,edge2Indexes) = sideLengths(2);
        positions(1,edge4Indexes) = sideLengths(1);
        positions(2,edge3Indexes) = sideLengths(2)*rand(1,numel(edge3Indexes));
        positions(2,edge4Indexes) = sideLengths(2)*rand(1,numel(edge4Indexes));
    case 'uniform'
        positions(1,:) = sideLengths(1)*rand(1,nPoints);
        positions(2,:) = sideLengths(2)*rand(1,nPoints);
end

end

