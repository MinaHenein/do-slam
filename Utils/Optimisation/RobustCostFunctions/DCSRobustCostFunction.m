%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/08/2018
% Contributors:
%--------------------------------------------------------------------------
%% g2o implementation
% //delta is used as $phi$
% void RobustKernelDCS::robustify(number_t e2, Vector3& rho) const
% {
%   const number_t& phi = _delta;
%   number_t scale = (2.0*phi)/(phi+e2);
%   if(scale>=1.0)
%     scale = 1.0;
% 
%   rho[0] = scale*e2*scale;
%   rho[1] = (scale*scale);
%   rho[2] = 0;
% }
%--------------------------------------------------------------------------
function system = DCSRobustCostFunction(config,system,edge)

iBlock = blockMap(system,edge,'edge');
error = system.b(iBlock);
delta = config.robustCostFunctionWidth;

phi = delta;
scale = (2*phi)/(phi+error);
  if(scale>=1)
    scale = 1;
  end 
  rho(1) = scale*error*scale;
  rho(2) = (scale*scale);
  rho(3) = 0;
  
  system.b(iBlock) = rho(1);
end