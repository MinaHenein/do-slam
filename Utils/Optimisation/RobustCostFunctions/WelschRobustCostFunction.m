%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelWelsch::robustify(number_t e2, Vector3& rho) const
% {
%   const number_t dsqr = _delta * _delta;
%   const number_t aux = e2 / dsqr;
%   const number_t aux2 = exp (-aux);
%   rho[0] = dsqr * (1. - aux2);
%   rho[1] = aux2;
%   rho[2] = -aux2 / dsqr;
% }
%--------------------------------------------------------------------------
function system = WelschRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

dsqr   = delta * delta;
aux    = error / dsqr;
aux2   =  exp(-aux);
rho(1) = dsqr * (1 - aux2);
rho(2) = aux2;
rho(3) = -aux2 / dsqr;

system.b(iBlock) = rho(1);

end