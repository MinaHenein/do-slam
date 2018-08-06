%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% void RobustKernelHuber::robustify(number_t e, Vector3& rho) const
% {
%   number_t dsqr = _delta * _delta;
%   if (e <= dsqr) { // inlier
%     rho[0] = e;
%     rho[1] = 1.;
%     rho[2] = 0.;
%   } else { // outlier
%     number_t sqrte = sqrt(e); // absolut value of the error
%     rho[0] = 2*sqrte*_delta - dsqr; // rho(e)   = 2 * delta * e^(1/2) - delta^2
%     rho[1] = _delta / sqrte;        // rho'(e)  = delta / sqrt(e)
%     rho[2] = - 0.5 * rho[1] / e;    // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
%   }
% }
%--------------------------------------------------------------------------
function system = HuberRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

dsqr = delta* delta;
if error <= dsqr % inlier
    rho(1) = error;
    rho(2) = 1;
    rho(3) = 0;
else %outlier
    sqrtError = sqrt(error);
    rho(1)    = 2 * sqrtError * delta - dsqr;
    rho(2)    = delta / sqrtError;
    rho(3)    = -0.5 * rho(1) / error;
end

system.b(iBlock) = rho(1);


end