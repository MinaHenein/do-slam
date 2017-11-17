function [Omega, omega] = computeOmega(config, fullGraph, graphUpdate)

Edge = graphUpdate.edges(1);
jacobian1 = Edge.jacobians(1);
jacobian2 = Edge.jacobians(2);
vertex1 = Edge.iVertices(1);
vertex2 = Edge.iVertices(2);
dimEdge = size(Edge.value);

nZeros = 0;
for i =  vertex1+1:vertex2-1
    type = fullGraph.iVertices(i).type;
    switch type
        case 'pose'
            nZeros = nZeros + config.dimPose;
        case 'point'
            nZeros = nZeros + config.dimPoint;
        %% TODO add more vertices cases
    end
end

% compute OMEGA
H = [jacobian1, zeros(dimEdge(1),nZeros), jacobian2];
Omega = H'*Edge.covariance\H;

% compute omega
residual = graphUpdate.computeResidual(config,Edge.index,Edge.value);
omega = -H*Edge.covariance^-0.5*residual;



end