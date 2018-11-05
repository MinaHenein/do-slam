function plotVelocityVertexes(graphN)
% plots individual points velocity estimates.
% Back-end not aware of ridig-body objects but rather treats moving objects
% as individual points linear velocities

figure;
for i=1:length(graphN.vertices)
    if strcmp(graphN.vertices(i).type,'velocity')
        velocityValue = graphN.vertices(i).value; 
        scatter3(velocityValue(1),velocityValue(2),velocityValue(3));
        hold on;
    end
end
xlabel('x');ylabel('y');zlabel('z');


end