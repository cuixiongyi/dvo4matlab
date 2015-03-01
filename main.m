% im1 is reference 
% im2 is current
global im1 im2 
global fx fy cx cy;
global kinectDepthPara
kinectDepthPara = 1.035 / 5000.0;

im1Raw = imread('1.png');
im2Raw = imread('2.png');
im1Raw = single(im1Raw);
im2Raw = single(im2Raw);
[depth1Raw, depth2Raw] = initConfig(im1Raw, im2Raw);

%% start each level
global fx_ fy_ cx_ cy_;

maxLevel = floor(log(size(im1Raw,1))-2);
fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
% z,  zdx, zdy
pointcloud1 = cell(1,maxLevel);
pointcloud2 = cell(1,maxLevel);
devrative1x = cell(1,maxLevel);
devrative1y = cell(1,maxLevel);
devrative2x = cell(1,maxLevel);
devrative2y = cell(1,maxLevel);
depth1 = cell(1,maxLevel);
depth2 = cell(1,maxLevel);
depth1{1} = depth1Raw; depth2{1} = depth2Raw;
%%
for ii = 1 : maxLevel
    imsize = size(depth1{ii});
   
    fx_ = fx*0.5^(ii-1); fy_ = fy*0.5^(ii-1); cx_ = cx*0.5^(ii-1); cy_ = cy*0.5^(ii-1);
    [pointcloud1{ii}, pointcloud2{ii}, devrative1x{ii}, devrative1y{ii}, devrative2x{ii}, devrative2y{ii}] = calculateElement(depth1{ii}, depth2{ii});
    if (ii ~= maxLevel)
        G = fspecial('gaussian',[3 3],1);
        Ig = imfilter(depth1{ii},G,'same');
        depth1{ii+1} = Ig(1:2:end,1:2:end);
        Ig = imfilter(depth2{ii},G,'same');
        depth2{ii+1} = Ig(1:2:end,1:2:end);
    end
end

%% calculate from corus to fine maxLevel -> 1
iterationInnerMax = 50;
itError = realmax;
% T = zeros(4); T(1,1) = 1; T(2,2) = 1; T(3,3) = 1; T(4,4) = 1;

meanVec = zeros(1,3);
% JacobianT;

variable = [X1, Y1, Z1, fxs, fys, cxs, cys, depth2dx, depth2dy, v1, v2, v3, w1, w2, w3];
%%