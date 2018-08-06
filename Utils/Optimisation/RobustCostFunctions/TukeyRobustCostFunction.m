%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelTukey::robustify(number_t e2, Vector3& rho) const
% {
%   const number_t e = sqrt(e2);
%   const number_t delta2 = _delta * _delta;
%   if (e <= _delta) {
%     const number_t aux = e2 / delta2;
%     rho[0] = delta2 * (1. - std::pow((1. - aux), 3)) / 3.;
%     rho[1] = std::pow((1. - aux), 2);
%     rho[2] = -2. * (1. - aux) / delta2;
%   } else {
%     rho[0] = delta2 / 3.;
%     rho[1] = 0;
%     rho[2] = 0;
%   }
% }
%--------------------------------------------------------------------------
function system = TukeyRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

error  = sqrt(error);
delta2 = delta * delta;
if (error <= delta)
    aux = error / delta2;
    rho(0) = delta2 * (1 - (1 - aux)^3) / 3;
    rho(1) = (1 - aux)^2;
    rho(2) = -2 * (1 - aux) / delta2;
else
    rho(0) = delta2 / 3;
    rho(1) = 0;
    rho(2) = 0;
end

system.b(iBlock) = rho(1);

end