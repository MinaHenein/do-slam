function [obj] = LMSolver_g2o(obj,config,graph0,measurementsCell)
%SOLVE solves system, calls nonlinear least squares solver and updates
%state until dX < threshold or max iterations completed
%   Detailed explanation goes here

done      = 0;
iteration = 1;
system = System(config,graph0,measurementsCell);
% covariance = system.covariance; %doesn't change in this function
FXCurrent = 0.5*sum(system.b).^2;   %initial value
tau = 1e-12;
eps1 = 1e-4;
eps2 = 1e-4;
% mu = 1e-4; %initial value
mu = tau*max(full(diag(system.A)));
v = 2;
g = system.A'*system.b;

%storing variables from each iteration
obj.dX      = [];
obj.graphs  = [graph0];
obj.systems = [system];

timeStart = tic;
while (~done)    
    %   build system & solve    
%     d = spdiags(mu*spdiags(system.H,0),0,size(system.H,1),size(system.H,2));
    d = mu*speye(size(system.H,1));
    dX = (system.H + d) \ system.c;
    dX = system.kPerp*dX;         
    dX = adjustUpdate(system,graph0,dX);
    
    %stopping
    normX = norm(cell2mat({graph0.vertices.value}'));
    if norm(dX) < eps2*(normX + eps2)
        done = 1;
    end
    
    %update system, get error
    graph0Update = graph0.updateVertices(config,system,dX);
    graph1 = graph0Update.updateEdges(config);
    FXTemp = 0.5*sum(graph1.constructResiduals(measurementsCell)).^2;  
    
    % update
    rho = FXCurrent - FXTemp;
    scale = sum(0.5*dX.*(mu*dX + system.c));
    if scale==0; scale = 1e-3; end;
    rho = rho/scale;
    if rho > 0 %(ie if errorTemp < errorCurrent -> error decreasing)
        %use update
        graph0 = graph1;
        system = System(config,graph1,measurementsCell);
        FXCurrent = FXTemp;
        g = system.A'*system.b;
        mu = mu*max(1/3,1-(2*rho-1)^3);
        v = 2;
        updateGraph = 1;
    else
        %dont use update
        mu = mu*v;
        v = 2*v;
        updateGraph = 0;
    end
    
    %display progress
    if config.displayProgress %&& updateGraph
        display(sprintf('It. %d:\t||dX|| = %.3e\t,\trho = %.3e\t,\tlambda = %.3e',iteration,norm(dX),rho,mu))
    end
    
    %   check termination criteria
    if (norm(g) < eps1) || (iteration >= obj.maxIterations) || norm(dX) > config.maxNormDX
        done = 1;
    else
        %   increment interation
        if updateGraph
            iteration = iteration + 1;
        end
    end
        
    %store
    obj.dX = [obj.dX dX];
    obj.graphs = [obj.graphs graph0];
    obj.systems = [obj.systems system];    
end

obj.solveTime = toc(timeStart);
obj.iterations = iteration;

end
