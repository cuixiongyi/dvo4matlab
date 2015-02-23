function [pointcloud1, pointcloud2, devrative1x, devrative1y, devrative2x, devrative2y] = calculateElement(depth1, depth2)

global fx_ fy_ cx_ cy_;
width = size(depth1, 2);
height = size(depth1, 1);

%% acquire 3d pointcloud

pointcloud1 = zeros([size(depth1), 3], 'single');
pointcloud2 = zeros([size(depth1), 3], 'single');

for ii = 1 : height
    for jj = 1 : width
        pointcloud1(ii,jj,:) = [(jj-cx_)/fx_*depth1(ii,jj), (ii-cy_)/fy_*depth1(ii,jj), depth1(ii,jj)];
    end
end
for ii = 1 : height
    for jj = 1 : width
        pointcloud2(ii,jj,:) = [(jj-cx_)/fx_*depth2(ii,jj), (ii-cy_)/fy_*depth2(ii,jj), depth2(ii,jj)];
    end
end
% imshow(pointcloud1);


%% build dirivative
devrative1x = zeros(size(depth1), 'single');
devrative1y = zeros(size(depth1), 'single');
devrative2x = zeros(size(depth1), 'single');
devrative2y = zeros(size(depth1), 'single');
for ii = 2 : height-1        
        devrative1y(ii,:) = (depth1(ii+1,:) - depth1(ii-1,:)) /2;
        devrative2y(ii,:) = (depth2(ii+1,:) - depth2(ii-1,:)) /2;
end
for ii = 2 : width-1
        devrative1x(:,ii) = (depth1(:,ii+1) - depth1(:,ii-1)) /2;
        devrative2x(:,ii) = (depth2(:,ii+1) - depth2(:,ii-1)) /2;
end
