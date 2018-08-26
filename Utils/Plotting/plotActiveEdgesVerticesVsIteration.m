function plotActiveEdgesVerticesVsIteration(solverEnd)

nActiveEdges = zeros(length(solverEnd.graphs),1);
for i=1:length(solverEnd.graphs)
    nActiveEdges(i) = sum(logical(cell2mat({solverEnd.graphs(1,i).edges.active}')));
end

nActiveVertices = zeros(length(solverEnd.graphs),1);
for i=1:length(solverEnd.graphs)
    k = zeros(solverEnd.graphs(1,i).nVertices,1);
    for j=1:solverEnd.graphs(1,i).nVertices
        if ~isempty(solverEnd.graphs(1,i).vertices(j).type)
            k(j) = 1;
        end
    end
    nActiveVertices(i) = sum(k);
end

nEdges = zeros(length(solverEnd.graphs),1);
for i=1:length(solverEnd.graphs)
    nEdges(i) = solverEnd.graphs(1,i).nEdges;
end

nVertices = zeros(length(solverEnd.graphs),1);
for i=1:length(solverEnd.graphs)
    nVertices(i) = solverEnd.graphs(1,i).nVertices;
end

iterations = 1:length(solverEnd.graphs);
figure;
plot(iterations,nEdges);
hold on
plot(iterations,nActiveEdges);
xlim([1 length(solverEnd.graphs)])
ylim([min(nActiveEdges)-10 max(nEdges)+10])
xlabel('iteration')
ylabel('number of edges')
legend('number of total graph edges','number of active graph edges')


figure;
plot(iterations,nVertices);
hold on
plot(iterations,nActiveVertices);
xlim([1 length(solverEnd.graphs)])
ylim([min(nActiveVertices)-10 max(nVertices)+10])
xlabel('iteration')
ylabel('number of vertices')
legend('number of total graph vertices','number of active graph vertices')

end