function [Omega, omega] = computeOmegas(config, fullGraph, Edge, residual)

vertex1 = Edge.iVertices(1);
jacobian1 = Edge.jacobians{1,1};
vertex2 = Edge.iVertices(2);
jacobian2 = Edge.jacobians{1,2};

edgeDim = size(Edge.value,1);
nZeros = 0;
for i =  vertex1+1:vertex2-1
    type = fullGraph.iVertices(i).type;
    switch type
        case 'pose'
            nZeros = nZeros + config.dimPose;
        case 'point'
            nZeros = nZeros + config.dimPoint;
        % TODO add more vertices cases
    end
end

%compute OMEGA
H = [jacobian1, zeros(edgeDim,nZeros), jacobian2];
Omega = H'*(Edge.covariance\H);

%compute omega
omega = -H'*(Edge.covariance^-0.5)*residual;

end