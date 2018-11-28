function plotDynamicPointReVisibility(sensor)

% find indexes for static and dynamic points
staticPointLogical = sensor.get('points').get('static');
if ~isempty(sensor.get('objects'))
    dynamicPointLogical  = ~staticPointLogical;
end
dynamicPointIndexes = find(dynamicPointLogical);

pointVisibility = sensor.get('pointVisibility');
a = zeros(nSteps-1,1);
for i=2:nSteps
    for j=dynamicPointIndexes
        if pointVisibility(j,i) && pointVisibility(j,i-1)
            a(i-1,1) = a(i-1,1)+1;
        end
    end
end
plot(2:nSteps,a)
xlim([2 nSteps])
yticks(0:10)
xlabel('time steps')
ylabel('# of points seen at both last and current time steps')

end