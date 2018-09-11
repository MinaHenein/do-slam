%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelPseudoHuber::robustify(number_t e2, Vector3& rho) const
% {
%   number_t dsqr = _delta * _delta;
%   number_t dsqrReci = 1. / dsqr;
%   number_t aux1 = dsqrReci * e2 + 1.0;
%   number_t aux2 = sqrt(aux1);
%   rho[0] = 2 * dsqr * (aux2 - 1);
%   rho[1] = 1. / aux2;
%   rho[2] = -0.5 * dsqrReci * rho[1] / aux1;
% }
%--------------------------------------------------------------------------
function errorNorm = PseudoHuberRobustCostFunction(config,system,edgeIndex)

iBlock = blockMap(system,edgeIndex,'edge');
error = norm(system.b(iBlock));
delta = config.robustCostFunctionWidth;

dsqr     = delta* delta;
dsqrReci = 1/dsqr;
aux1     = dsqrReci * error + 1;
aux2     = sqrt(aux1);
rho(1)   = 2 * dsqr * (aux2-1);
rho(2)   = 1 / aux2;
rho(3)   = -0.5 * dsqrReci * rho(1) / aux1;

errorNorm = rho(1);

end