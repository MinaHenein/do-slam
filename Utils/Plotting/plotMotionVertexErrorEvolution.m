function plotMotionVertexErrorEvolution(solver, objectsGTMotion, objectsGTFrames)

 nObjects = size(objectsGTMotion,2);

relativeTranslation = [];
relativeRotation = [];
f_deg_per_rad = 180/pi;

nObjectsSoFar = 0;
objectStartFrames = zeros(nObjects,1);
for i=1:length(solver)-1
    graph = solver(i).graphs(end);
    motionVertices = [graph.vertices(graph.identifyVertices('SE3Motion')).value];
    if size(motionVertices,2) > nObjectsSoFar
        objectStartFrames(size(motionVertices,2)) = i;
    end
    nObjectsSoFar = size(motionVertices,2);    
end

for i=1:nObjects
    objectStartFrame = objectStartFrames(i);
    objectGTMotion = cell2mat(objectsGTMotion(i));
    for j= objectStartFrame:length(solver)-1
        graph = solver(j).graphs(end);
        motionVertices = [graph.vertices(graph.identifyVertices('SE3Motion')).value];
        objectMotion = motionVertices(:,i);
        indx = find(cell2mat(objectsGTFrames(i))==j+334);
        if j > size(objectGTMotion,2) 
            relativeMotion = AbsoluteToRelativePoseR3xso3(objectMotion,objectGTMotion(:,end));
        else
            relativeMotion = AbsoluteToRelativePoseR3xso3(objectMotion,objectGTMotion(:,indx));
        end
        relativeTranslation(j,i) = norm(relativeMotion(1:3));
        relativeRotation(j,i) = wrapToPi(norm(relativeMotion(4:6)))*f_deg_per_rad;
    end
end
        
colors = {'red','blue','black','green','magenta','sapphire','leather','swamp','light bluish green',...
    'butterscotch','cinnamon','radioactive green','chartreuse'}; 

%translation
figure;
for i=1:nObjects
    k = find(relativeTranslation(:,i)); 
    plot(k(1):length(solver)-1,relativeTranslation(k(1):end,i),'Color',rgb(colors(i)))    
    hold on
end

legend('Object 1','Object 2','Object 3','Object 4','Object 5','Object 6',...
    'Object 7','Object 8','Object 9','Object 10','Object 11','Object 12');
xlabel('time step')
ylabel('Translation error (m)')
title({'Evolution of object motion error','translational component'})

%rotation
figure;
for i=1:nObjects
    k = find(relativeRotation(:,i)); 
    plot(k(1):length(solver)-1,relativeRotation(k(1):end,i),'Color',rgb(colors(i)))    
    hold on
end
legend('Object 1','Object 2','Object 3','Object 4','Object 5','Object 6',...
    'Object 7','Object 8','Object 9','Object 10','Object 11','Object 12');
xlabel('time step')
ylabel('Rotation error (°)')
title({'Evolution of object motion error','rotational component'})
end
