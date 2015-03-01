function T = xpose2T(xpose)
T = zeros(4);
T(4,4) = 1;
% rx = xpose(4);
% ry = xpose(5);
% rz = xpose(6);
%     Rx = [1, 0, 0;
%       0, cos(rx), -sin(rx);
%       0, sin(rx), cos(rx)];
%   Ry = [cos(ry), 0, sin(ry);
%         0, 1, 0;
%         -sin(ry), 0, cos(ry)];
%     Rz =[cos(rz), -sin(rz), 0;
%          sin(rz), cos(rz), 0;
%          0, 0, 1];
%      T(1:3,1:3) = Rz*Ry*Rx;
%      T(1:3,4) = [xpose(1),xpose(2),xpose(3)];
     
     
%% use Rodrigues
T(1:3,1:3) = Rodrigues(xpose(4:6));
T(1:3,4) = [xpose(1),xpose(2),xpose(3)]';
     
T = twistexp(xpose);
end