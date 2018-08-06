%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelGemanMcClure::robustify(number_t e2, Vector3& rho) const
% {
%   const number_t aux = _delta / (_delta + e2);
%   rho[0] = e2 * aux;
%   rho[1] = aux * aux;
%   rho[2] = -2. * rho[1] * aux;
% }
%--------------------------------------------------------------------------
function system = GemanMcClureRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

aux    = delta / (delta + error);
rho(1) = error * aux;
rho(2) = aux * aux;
rho(3) = -2 * rho(1) * aux;

system.b(iBlock) = rho(1);

end