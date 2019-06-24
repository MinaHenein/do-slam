%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [] = pbar(k,N,nbars,msg,clk)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(nargin<3)||isempty(nbars); nbars = 79;    end
if(nargin<4)||isempty(msg);   msg   = [];    end
if(nargin<5)||isempty(clk);   clk   = 'off'; end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if(k~=1)
    fprintf(repmat('\b',1,(nbars+3)*3))
end

num = num2str(ceil(k/N*100));

if (numel(num)==1)
    num = strcat('==',num);
elseif(numel(num)==2)
    num = strcat('=',num);
end

num = strcat(num,'% ');

line  = repmat('=',1,nbars+2);
line(floor(numel(line)/2)-1:floor(numel(line)/2)+2)=num;

if(~isempty(msg))
    line(2:numel(msg)+3) = [' ' msg ' '];
end

if(strcmpi(clk,'on'))
    line(end-11:end-2) = [' ' TimeStamp('on') ' '];
end

factor = ceil(k/N.*nbars);
disp(strcat(line));
disp(strcat('[',repmat('|',1,factor),repmat('-',1,nbars-factor),']'));
disp(repmat('=',1,nbars+2));

if(k==N)
    pause(0.5); fprintf(repmat('\b',1,(nbars+3)*3))
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%