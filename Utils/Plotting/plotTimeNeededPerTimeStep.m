function plotTimeNeededPerTimeStep(solver)

time = zeros(1,length(solver));
for i =1:length(solver)
    time(i)= solver(i).solveTime;
end
figure;
plot(1:length(solver),time)
xlim([1 length(solver)])
ylim([0 max(time)+2])
xlabel('time step')
ylabel('solver time (s)')
title('Solver time needed vs time step')
    
end