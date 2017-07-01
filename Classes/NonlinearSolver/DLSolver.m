function [obj] = DLSolver(obj,config,graph0,measurementsCell)
%Powell's Dog Leg algorithm

done      = 0;
iteration = 1;

%parameters - how to set these???
delta0 = 0.2;
eta1 = 0.01;
eta2 = 0.2;
gamma1 = 0.1;
gamma2 = 1.5;


%initialise
delta = delta0;
graphCurrent  = graph0;
systemCurrent = System(config,graph0,measurementsCell);

while (~done)    
    g = (systemCurrent.A)'*systemCurrent.b;
    [Q,R] = qr(systemCurrent.A);
    R = R(1:length(g),:);
    de = Q'*systemCurrent.b;
    d = de(1:length(g));
    
    if (condest(R) > 1e-12)
        hGN = R \ d;
        alpha = sum(g.^2)/sum((systemCurrent.A*g).^2);
        hGD = -alpha*g;
        h = computeDogLeg(hGN,hGD,delta);
    else    
        if sum((systemCurrent.A*g).^2) > 0
            K = min(delta/norm(g),alpha);
        else
            K = delta/norm(g);
        end
        h = -K*g;
    end
    
    graphUpdate = graphCurrent.updateVertices(config,systemCurrent,h);
    graphUpdate = graphUpdate.updateEdges(config); 
    systemUpdate = System(config,graphUpdate,measurementsCell);
    
    B = systemCurrent.A'*systemCurrent.A;
    ared = norm(systemCurrent.b) - norm(systemUpdate.b);
    pred = -norm(g'*h + 0.5*h'*B*h);
    rho = ared/pred;
    if rho > eta1
        graphCurrent  = graphUpdate;
        systemCurrent = systemUpdate;
        iteration = iteration + 1;
    end
    
    delta = updateDelta(rho,delta,eta1,eta2,gamma1,gamma2);
    
    display(sprintf('It. %d:\t||dX|| = %.3e\t,\trho = %.3e\t,\tdelta = %.3e',iteration,norm(h),rho,delta))
    
    %store
    obj.dX = [obj.dX h];
    obj.graphs = [obj.graphs graphCurrent];
    obj.systems = [obj.systems systemCurrent]; 
    
    %stopping criteria
    if  norm(h) < obj.threshold || (iteration >= obj.maxIterations) || norm(h) > config.maxNormDX
        done = 1;
    end
    
end

end


%%
function [hDL] = computeDogLeg(hGN,hGD,delta)
    if norm(hGN) <= delta
        hDL = hGN;
    elseif norm(hGD) >= delta
        hDL = delta/norm(hGD)*hGD;
    else
        v = hGN - hGD;
        beta = (-hGD'*v + sqrt((hGD'*v)^2 + (delta^2 - sum(hGD.^2))*sum(v.^2)))/sum(v.^2);
        hDL = hGD + beta*(hGN - hGD);
    end
end

function [delta] = updateDelta(rho,delta,eta1,eta2,gamma1,gamma2)
    if rho >= eta2
        delta = gamma2*delta;
    elseif rho < eta1
        delta = gamma1*delta;
    end
end