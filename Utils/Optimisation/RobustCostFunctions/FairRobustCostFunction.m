%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelFair::robustify(number_t e2, Vector3& rho) const
% {
%   const number_t sqrte = sqrt(e2);
%   const number_t aux = sqrte / _delta;
%   rho[0] = 2. *  _delta * _delta * (aux - log(1. + aux));
%   rho[1] = 1. / (1. + aux);
%   rho[2] = - 0.5 / (sqrte * (1. + aux));
% }
%--------------------------------------------------------------------------
function system = FairRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

sqrtError = sqrt(error);
aux       = sqrtError / delta;
rho(1)    = 2 * delta * delta * (aux - log(1 + aux));
rho(2)    = 1 / (1 + aux);
rho(3)    = -0.5 / (sqrtError * (1 + aux));

system.b(iBlock) = rho(1);

end