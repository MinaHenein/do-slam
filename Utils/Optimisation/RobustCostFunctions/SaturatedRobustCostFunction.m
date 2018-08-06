%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelSaturated::robustify(number_t e2, Vector3& rho) const
% {
%   number_t dsqr = _delta * _delta;
%   if (e2 <= dsqr) { // inlier
%     rho[0] = e2;
%     rho[1] = 1.;
%     rho[2] = 0.;
%   } else { // outlier
%     rho[0] = dsqr;
%     rho[1] = 0.;
%     rho[2] = 0.;
%   }
% }
%--------------------------------------------------------------------------
function system = SaturatedRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

dsqr = delta * delta;
if (error <= dsqr) %inlier
    rho(1) = error;
    rho(2) = 1;
    rho(3) = 0;
else %outlier
    rho(1) = dsqr;
    rho(2) = 0;
    rho(3) = 0;
end

system.b(iBlock) = rho(1);

end