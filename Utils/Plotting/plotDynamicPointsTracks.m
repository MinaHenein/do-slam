function plotDynamicPointsTracks(graphN, points_T)


 SE3MotionVertices = [graphN.identifyVertices('SE3Motion')];
 pointVertices = [graphN.vertices(graphN.identifyVertices('point'))];   
 edgesConnectedToMotionVertices = [graphN.vertices(SE3MotionVertices).iEdges];
 dynamicPointsMotionIndices = [graphN.edges(edgesConnectedToMotionVertices).iVertices]';
 dynamicPointsIndices = setdiff(dynamicPointsMotionIndices,SE3MotionVertices);

 objectPoints = {};
 
 posesIndices = [graphN.vertices(graphN.identifyVertices('pose')).index];
 
 for i=1:length(dynamicPointsIndices)
     dynamicPointEdges = [graphN.vertices(dynamicPointsIndices(i)).iEdges];
     pointsMotionVertexIndices = [graphN.edges(dynamicPointEdges(end)).iVertices]';
     objectMotionIndex = pointsMotionVertexIndices(end);
     idx = find(SE3MotionVertices == objectMotionIndex);
     if length(objectPoints)>= idx
        objectPoints{idx,1}= [objectPoints{idx,1},dynamicPointsIndices(i)];
     else
        objectPoints{idx,1}= dynamicPointsIndices(i);
     end
     
 end
 
 points = [pointVertices.value]; 
 for i=1:size(points,2)
     points(:,i) = RelativeToAbsolutePositionR3xso3(points_T,points(:,i));
 end
 
 
colors = {'red','blue','green','black','magenta','sapphire','leather','swamp','light bluish green',...
    'butterscotch','cinnamon','radioactive green','chartreuse'}; 
 
figure; 
for i=1:size(points,2)
     if ismember(pointVertices(i).index,dynamicPointsIndices)       
         for j=1:length(objectPoints)
            if ismember(pointVertices(i).index,[objectPoints{j}])
                objectID = j;
            end
         end
         plot3(points(1,i),points(2,i),points(3,i),'x','Color',rgb(colors(objectID)),'MarkerSize',5)
         hold on
     end
 end
 
 
end