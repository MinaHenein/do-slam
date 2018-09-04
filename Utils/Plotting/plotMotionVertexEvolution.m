function plotMotionVertexEvolution(solver, nObjects)

relativeTranslation = zeros(length(solver)-1,nObjects);
relativeRotation = zeros(length(solver)-1,nObjects);
f_deg_per_rad = 180/pi;

for i=1:length(solver)-1
    graph = solver(i).graphs(end);
    nextGraph = solver(i+1).graphs(end);
    motionVertices = [graph.vertices(graph.identifyVertices('SE3Motion')).value];
    nextMotionVertices = [nextGraph.vertices(nextGraph.identifyVertices('SE3Motion')).value];
    for j=1:size(motionVertices,2)
        objectMotion = motionVertices(:,j);
        objectNextMotion = nextMotionVertices(:,j);
        relativeMotion = AbsoluteToRelativePoseR3xso3(objectMotion,objectNextMotion);
        relativeTranslation(i,j) = norm(relativeMotion(1:3));
        relativeRotation(i,j) = wrapToPi(norm(relativeMotion(4:6)))*f_deg_per_rad;
    end 
end

%translation
figure;
for i=1:nObjects
    plot(1:length(solver)-1,relativeTranslation(:,i))    
    hold on
end
legend('Object 1','Object 2','Object 3','Object 4','Object 5');
xlabel('time step')
ylabel('Translation difference norm (m)')
title({'Evolution of object motion vertices','translational component'})

%rotation
figure;
for i=1:nObjects
    plot(1:length(solver)-1,relativeRotation(:,i))    
    hold on
end
legend('Object 1','Object 2','Object 3','Object 4','Object 5');
xlabel('time step')
ylabel('Rotation difference norm (°)')
title({'Evolution of object motion vertices','rotational component'})
end
