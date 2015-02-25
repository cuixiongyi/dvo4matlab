function [jw,jz] = computeJacobianOfProjectionAndTransformation(p, xpose,JZ2, variable, devrative2x,devrative2y)
p = reshape(p,[1,3]);
z = 1/p(3);
z_sqr = z^2;
x = p(1);
y = p(2);
jw = zeros(2,6);
% jw(1, 1) =  z;
%   jw(1, 2) =  0.0;
%   jw(1, 3) = -x * z_sqr;
%   jw(1, 4) = jw(1, 3) * y;%//j(0, 3) = -x * y * z_sqr;
%   jw(1, 5) = 1.0 - jw(1, 3) * x;%//j(0, 4) =  (1.0 + x * x * z_sqr);
%   jw(1, 6) = -y * z;
% 
%   jw(2, 1) =  0.0;
%   jw(2, 2) =  z;
%   jw(2, 3) = -y * z_sqr;
%   jw(2, 4) = -1.0 + jw(2, 3) * y; %//j(1, 3) = -(1.0 + y * y * z_sqr);
%   jw(2, 5) =  x*y*z_sqr; %//j(1, 4) =  x * y * z_sqr;
%   jw(2, 6) =  x * z;
%   
%   jz(1) = 0.0;
%   jz(2) = 0.0;
%   jz(3) = 1.0;
%   jz(4) = y;
%   jz(5) = x;
%   jz(6) = 0.0;
% 
%   nanJz = sum(isnan(jz));
%   nanJw = sum(isnan(jw));
%   if nanJz(1) > 0 || nanJw(1) > 0
%       'nan in Jacobian'
%       jw = zeros(2,6);
%       jz = zeros(1,6);
%   end

  global fx_ fy_ cx_ cy_ ;
    syms X1 Y1 Z1 v1 v2 v3 w1 w2 w3 Xihat Xi fxs fys cxs cys depth2dx depth2dy
  if xpose(1)* xpose(2)*xpose(3)*xpose(4)*xpose(5)*xpose(6) ~= 0
    jz = double(subs(JZ2, variable, {p(1), p(2), p(3), fx_, fy_, cx_, cy_, devrative2x, devrative2y, xpose(1), xpose(2),xpose(3),xpose(4),xpose(5),xpose(6) }));
  end
  
end