
function Chol_ela

A=[1, 0 0 0 0 0 0
-1, 1, 0 0 0 0 0
0 -1, 1, 0 0 0 0
1, 0 0 -1, 0 0 0
0  0 -1 1 0 0 0
0  1  0 0 -1 0 0
0  0  0 0  -1 1 0
0  0  0  0  0 -1 1];

%A=[1 0 0 0;-1 1 0 0 ;0 -1 1 0; 0 0 -1 1;1 0 0 -1];

Lambda=A'*A;
sL=size(Lambda,1);
R=zeros(sL,sL);

for j=1:sL
    j
    for i=1:sL
        i
        if (R(i,i))
            R(i,j)=(Lambda(i,j)-R(:,i)'*R(:,j))/R(i,i);
        else
            R(i,j)=0;
        end
        R
    end
    R(j,j)=sqrt(Lambda(j,j)-R(:,j)'*R(:,j));
    R
end

norm(R-chol(Lambda))
end

function [L] = cholupdate(L,x)
    p = length(x);
    x = x';
    for k=1:p
        r = sqrt(L(k,k)^2 + x(k)^2);
        c = r / L(k, k);
        s = x(k) / L(k, k);
        L(k, k) = r;
        L(k,k+1:p) = (L(k,k+1:p) + s*x(k+1:p)) / c;
        x(k+1:p) = c*x(k+1:p) - s*L(k, k+1:p);
    end
end