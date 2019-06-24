%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [header] = TimeStamp(opt)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(nargin<1); opt = []; end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
clk = round(clock);
if (clk(4)<10)
    hr      = ['0' num2str(clk(4))];
else
    hr      = num2str(clk(4)) ;
end

if (clk(5)<10)
    mn      = ['0' num2str(clk(5))];
else
    mn      = num2str(clk(5));
end

if (clk(6)<10)
    sc      = ['0' num2str(clk(6))];
else
    sc      = num2str(clk(6)) ;
end

if (strcmpi(opt,'on'))
    header     = [hr ':' mn ':' sc];
else
    yr         = num2str(clk(1));
    
    if (clk(2)<10)
        mt     = ['0' num2str(clk(2))];
    else
        mt     = num2str(clk(2)) ;
    end
    
    if (clk(3)<10)
        dy     = ['0' num2str(clk(3))];
    else
        dy     = num2str(clk(3)) ;
    end
    
    header     = [yr mt dy hr mn sc];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%