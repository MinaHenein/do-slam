function plotActiveEdgesVerticesVsIteration(solver)

nActiveEdges = zeros(length(solver),1);
for i=1:length(solver)
    nActiveEdges(i) = sum(logical(cell2mat({solver(i).graphs(end).edges.active}')));
end

nActiveVertices = zeros(length(solver),1);
for i=1:length(solver)
    k = zeros(solver(i).graphs(end).nVertices,1);
    for j=1:solver(i).graphs(end).nVertices
        if ~isempty(solver(i).graphs(end).vertices(j).type)
            k(j) = 1;
        end
    end
    nActiveVertices(i) = sum(k);
end

nEdges = zeros(length(solver),1);
for i=1:length(solver)
    nEdges(i) = solver(i).graphs(end).nEdges;
end

nVertices = zeros(length(solver),1);
for i=1:length(solver)
    nVertices(i) = solver(i).graphs(end).nVertices;
end

iterations = 1:length(solver);
figure;
plot(iterations,nEdges);
hold on
plot(iterations,nActiveEdges);
xlim([1 length(solver)])
ylim([min(nActiveEdges)-10 max(nEdges)+10])
xlabel('time step')
ylabel('number of edges')
legend('number of total graph edges','number of active graph edges')


figure;
plot(iterations,nVertices);
hold on
plot(iterations,nActiveVertices);
xlim([1 length(solver)])
ylim([min(nActiveVertices)-10 max(nVertices)+10])
xlabel('time step')
ylabel('number of vertices')
legend('number of total graph vertices','number of active graph vertices')

end