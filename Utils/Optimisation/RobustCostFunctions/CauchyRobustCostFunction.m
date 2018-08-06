%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelCauchy::robustify(number_t e2, Vector3& rho) const
% {
%   number_t dsqr = _delta * _delta;
%   number_t dsqrReci = 1. / dsqr;
%   number_t aux = dsqrReci * e2 + 1.0;
%   rho[0] = dsqr * log(aux);
%   rho[1] = 1. / aux;
%   rho[2] = -dsqrReci * std::pow(rho[1], 2);
% }
%--------------------------------------------------------------------------
function system = CauchyRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

dsqr     = delta* delta;
dsqrReci = 1/dsqr;
aux      = dsqrReci * error + 1;
rho(1)   = dsqr * log(aux);
rho(2)   = 1 / aux;
rho(3)   = - dsqrReci * rho(1)^2;

system.b(iBlock) = rho(1);

end