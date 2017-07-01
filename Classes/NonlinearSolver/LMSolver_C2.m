function [obj] = LMSolver_C2(obj,config,graph0,measurementsCell)
%SOLVE solves system, calls nonlinear least squares solver and updates
%state until dX < threshold or max iterations completed
%   Detailed explanation goes here

done      = 0;
iteration = 1;
threshold = 1e-3;
% lambdaUp = 15;
% lambdaDown = 5;
lambdaUp = 18;
lambdaDown = 5;

%initial lambda
system = System(config,graph0,measurementsCell);
% lambda = full(mean(diag(system0.H)));
lambda = 1e-3;

%initial chi squared
currentChi = system.chiSquaredError;

%storing variables from each iteration
obj.dX      = [];
obj.graphs  = [graph0];
obj.systems = [system];

timeStart = tic;
while (~done)    
    %   build & solve system
    d = spdiags(lambda*spdiags(system.H,0),0,size(system.H,1),size(system.H,2));
    dX = (system.H + d) \ system.c;
    dX = system.kPerp*dX;         
    dX = adjustUpdate(system,graph0,dX);
    
    %update system, get error
    graph0Update = graph0.updateVertices(config,system,dX);
    graph1 = graph0Update.updateEdges(config);
    systemTemp = System(config,graph1,measurementsCell); %pass argument - only need residual
    tempChi = systemTemp.chiSquaredError;
    
    %update 
    rho = currentChi - tempChi;
    if rho > 0
        %use update
        lambda = lambda/lambdaDown;
        currentChi = tempChi;
        graph0 = graph1;
        system = systemTemp;
        updateGraph = 1;
    else
        %dont use update
        lambda = lambda*lambdaUp;
        currentChi = tempChi;
        graph0 = graph1;
        system = systemTemp;
        updateGraph = 0;
    end
    
    %display progress
    if config.displayProgress
        display(sprintf('It. %d:\t||dX|| = %.3e\t,\trho = %.3e\t,\tlambda = %.3e',iteration,norm(dX),rho,lambda))
    end
    
    %   check termination criteria
    if (norm(dX) < threshold) || (iteration >= obj.maxIterations) || norm(dX) > config.maxNormDX
%     if (rho < threshold && rho > 0) || (iteration >= obj.maxIterations) || norm(dX) > config.maxNormDX
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
