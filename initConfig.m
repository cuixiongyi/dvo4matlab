function [depth1Raw, depth2Raw] = initConfig(im1Raw, im2Raw)
global kinectDepthPara;
global fx fy cx cy;
fx = 517.3; fy = 516.5; cx = 318.6; cy = 255.3;
width = size(im1Raw, 2);
height = size(im1Raw, 1);

%% acquire 3d pointcloud

depth1Raw = single(zeros(size(im1Raw)));
depth2Raw = single(zeros(size(im1Raw)));
for ii = 1 : height
    for jj = 1 : width
        if im1Raw(ii,jj) ~= 0
            depth_Tmp = im1Raw(ii,jj)*kinectDepthPara;
        else
            depth_Tmp = nan;
        end
        depth1Raw(ii,jj) = depth_Tmp;
        
    end
end
for ii = 1 : height
    for jj = 1 : width
        if im2Raw(ii,jj) ~= 0
            depth_Tmp = im2Raw(ii,jj)*kinectDepthPara;
        else
            depth_Tmp = nan;
        end
        depth2Raw(ii,jj) = depth_Tmp;
    end
end
