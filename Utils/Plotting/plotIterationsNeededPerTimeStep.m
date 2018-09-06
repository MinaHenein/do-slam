function plotIterationsNeededPerTimeStep(solver)

it = zeros(1,length(solver));
for i =1:length(solver)
    it(i)= solver(i).iterations;
end
figure;
plot(1:length(solver),it)
xlim([1 length(solver)])
xlim([0 1000])
xlabel('time step')
ylabel('number of iterations')
title('Number of iterations needed vs time step')
    
end