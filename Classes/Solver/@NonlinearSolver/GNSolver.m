function [obj] = GNSolver(obj,config,graph0,measurementsCell)
%GNSOLVER solves nonlinear least squares problem with Gauss-Newton method
%   while ||dX|| <= threshold:
%       construct linear system from graph
%       solve linear system for dX
%       update vertices with dX
%       update edges
%       check termination criteria

%%
done = 0;
iteration = 1;
dXPrev = inf;
weight = 1;

obj.dX      = [];
obj.graphs  = [graph0];
obj.systems = [System(config,graph0,measurementsCell,weight)];

timeStart = tic;
while (~done)
    %   build system
    system = System(config,graph0,measurementsCell,weight);
    
    %   solve linear system       
    if config.displaySPPARMS
        spparms('spumoni',2)
    end
    dX = system.H \ system.c;
%     dX = constrainedOptimisation(graph0,system);
    
    %adjustments based on plane parameterisation
    switch config.planeNormalParameterisation
        case 'S2'           
            dX = system.kPerp*dX;         
            %dX for plane parameters projected onto tangent space
            dX = adjustUpdate(system,graph0,dX);
        case 'R3'
            %do nothing
    end

    %something bad happened
    assert(isempty(find(isnan(dX),1)),'Solution contains NaN')
    
    %   update graph vertices with dX, then graph edges
    graph0Update = graph0.updateVertices(config,system,dX);
    graph1 = graph0Update.updateEdges(config);
    
    %display progress
    if config.displayProgress
        display(sprintf('Iteration %d:\t||dX|| = %.3e',iteration,norm(dX)))
    end
    
    %   check termination criteria
    if (norm(dX) <= obj.threshold) || (iteration >= obj.maxIterations) || ...
       (norm(dX) > config.maxNormDX && norm(dX) > norm(dXPrev))
        done = 1;
    else
        %   updated graph is next linearisation point
        graph0 = graph1;
        %   increment interation
        iteration = iteration + 1;
    end
        
    %store
    dXPrev = dX;
    obj.dX = [obj.dX dX];
    obj.graphs = [obj.graphs graph0];
    obj.systems = [obj.systems system];    
end

obj.solveTime = toc(timeStart);
obj.iterations = iteration;

end
