function plotMotionVertexErrorEvolutionFromGraphFile(config,resultsFileName,nSteps,startFrame,objectsGTMotion,objectsGTFrames)

nObjects = size(objectsGTMotion,2);

relativeTranslation = [];
relativeRotation = [];
f_deg_per_rad = 180/pi;

nObjectsSoFar = 0;
objectStartFrames = zeros(nObjects,1);
for i=1:nSteps-1
    resultsCell = graphFileToCell(config,strcat(resultsFileName,num2str(i),'.graph'));
    graph = Graph(config,resultsCell);
    
    motionVertices = [graph.vertices(graph.identifyVertices('SE3Motion')).value];
    for j = size(motionVertices,2):-1:nObjectsSoFar+1
        objectStartFrames(j) = i;
    end
    nObjectsSoFar = size(motionVertices,2);    
end

for i=1:nObjects
    objectStartFrame = objectStartFrames(i);
    objectGTMotion = cell2mat(objectsGTMotion(i));
    if ~isempty(objectGTMotion)
        for j= objectStartFrame:nSteps-1
            resultsCell = graphFileToCell(config,strcat(resultsFileName,num2str(j),'.graph'));
            graph = Graph(config,resultsCell);
            motionVertices = [graph.vertices(graph.identifyVertices('SE3Motion')).value];
            objectMotion = motionVertices(:,i);
            indx = find(cell2mat(objectsGTFrames(i))==j+startFrame);
            if j > size(objectGTMotion,2)
                relativeMotion = AbsoluteToRelativePoseR3xso3(objectMotion,objectGTMotion(:,end));
            elseif ~isempty(indx)
                relativeMotion = AbsoluteToRelativePoseR3xso3(objectMotion,objectGTMotion(:,indx));
            end
            relativeTranslation(j,i) = norm(relativeMotion(1:3));
            relativeRotation(j,i) = wrapToPi(norm(relativeMotion(4:6)))*f_deg_per_rad;
        end
    end
end
        
colors = {'red','blue','black','green','magenta','sapphire','leather','swamp','light bluish green',...
    'butterscotch','cinnamon','radioactive green','chartreuse'}; 

%translation
figure;
for i=1:nObjects
    objectGTMotion = cell2mat(objectsGTMotion(i));
    if ~isempty(objectGTMotion)
        k = find(relativeTranslation(:,i)); 
        plot(k(1):nSteps-1,relativeTranslation(k(1):end,i),'Color',rgb(colors(i)))    
        hold on
    end
end

legend('Object 1','Object 2','Object 3','Object 4','Object 5','Object 6',...
    'Object 7','Object 8','Object 9','Object 10','Object 11','Object 12');
xlabel('time step')
ylabel('Translation error (m)')
title({'Evolution of object motion error','translational component'})

%rotation
figure;
for i=1:nObjects
    objectGTMotion = cell2mat(objectsGTMotion(i));
    if ~isempty(objectGTMotion)
        k = find(relativeRotation(:,i)); 
        plot(k(1):nSteps-1,relativeRotation(k(1):end,i),'Color',rgb(colors(i)))    
        hold on
    end
end
legend('Object 1','Object 2','Object 3','Object 4','Object 5','Object 6',...
    'Object 7','Object 8','Object 9','Object 10','Object 11','Object 12');
xlabel('time step')
ylabel('Rotation error (°)')
title({'Evolution of object motion error','rotational component'})
end
