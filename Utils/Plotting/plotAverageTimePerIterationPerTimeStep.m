function plotAverageTimePerIterationPerTimeStep(solver)

averageTime = zeros(1,length(solver));
for i =1:length(solver)
    averageTime(i)= solver(i).solveTime/solver(i).iterations;
end
figure;
plot(1:length(solver),averageTime)
xlim([1 length(solver)])
ylim([0 max(averageTime)+2])
xlabel('time step')
ylabel('average time per iteration (s)')
title('Average time needed per iteration vs time step')
    
end