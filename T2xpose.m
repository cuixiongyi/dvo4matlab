function [xpose]= T2xpose(T)

% This function return the rotation along x,y and z direction from a 
% Rotation Matrix

%Inputs:
    % R= 3x3 Rotation Matrix
%Outputs:
    % rx= Rotation along x direction in radians
    % ry= Rotation along y direction in radians
    % rz= Rotation along z direction in radians
    
%     R =
%  
% [                           cos(ry)*cos(rz),                          -cos(ry)*sin(rz),          sin(ry)]
% [ cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry), cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz), -cos(ry)*sin(rx)]
% [ sin(rx)*sin(rz) - cos(rx)*cos(rz)*sin(ry), cos(rz)*sin(rx) + cos(rx)*sin(ry)*sin(rz),  cos(rx)*cos(ry)]

% Author : Sandeep Sasidharan
% http://sandeepsasidharan.webs.com
R = T(1:3,1:3);
% xpose = zeros(1,6);
% xpose(5)=asin(R(1,3));
% xpose(6)=acos(R(1,1)/cos(xpose(5)));
% xpose(4)=acos(R(3,3)/cos(xpose(5)));
% xpose(1) = T(1,4);
% xpose(2) = T(2,4);
% xpose(3) = T(3,4);
xpose = zeros(1,6);
xpose(4:6) = Rodrigues(T(1:3,1:3));
xpose(1:3) = [T(1,4), T(2,4), T(3,4)];


twisttmp = twistlog(T);
xpose(1:3) = [twisttmp(1,4), twisttmp(2,4), twisttmp(3,4)];
xpose(4) = -twisttmp(1,2);
xpose(5) =  twisttmp(1,3);
xpose(6) = -twisttmp(2,3);
xpose = xpose';
end