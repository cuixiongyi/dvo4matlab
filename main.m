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
        depth1{ii+1} = depth1{ii}(1:2:end,1:2:end);
        depth2{ii+1} = depth2{ii}(1:2:end,1:2:end);
    end
end

%% calculate from corus to fine maxLevel -> 1
iterationInnerMax = 50;
itError = realmax;
% T = zeros(4); T(1,1) = 1; T(2,2) = 1; T(3,3) = 1; T(4,4) = 1;

meanVec = zeros(1,3);
%%
residual = cell(1,maxLevel);
residualCorres = cell(1,maxLevel);
count = cell(1,maxLevel);
weight = cell(1,maxLevel);
projectedPointcloud = cell(1,maxLevel);
precision = eye(3);
mean2 = [0,0,0];
lsList = cell(1,maxLevel);
% x is se3 transformation from reference to current, is the inverse
% transformation T corresponding SE3 to x
x = zeros(1,6)+0.0001;
T = xpose2T(x);
xpose = T2xpose(T)
% output JZ2  is the Jacobian
JacobianT;
variable = [X1, Y1, Z1, fxs, fys, cxs, cys, depth2dx, depth2dy, v1, v2, v3, w1, w2, w3];
xRecord = cell(1,maxLevel);
xLastTime = x;
for ww = maxLevel : -1 : 3
    iterationInner = 0;
    fx_ = fx*0.5^(ww-1); fy_ = fy*0.5^(ww-1); cx_ = cx*0.5^(ww-1); cy_ = cy*0.5^(ww-1);
    likilihoodLast = realmax;
    iteration = 0;
    ifHaveNewx = false;
    while (1)
        A = zeros(6);
        b = zeros(6,1);
        if ifHaveNewx
            T = xpose2T(x)*xpose2T(xpose);
            xLastTime = xpose;
            xpose = T2xpose(T)
        end
        ifHaveNewx = false;
%         xpose = [0,0,0,0,0,0];
        [count{ww}, residual{ww}, residualCorres{ww}, projectedPointcloud{ww}] = computeResidual(T, depth1{ww}, depth2{ww}, pointcloud1{ww}, pointcloud2{ww}, devrative1x{ww}, devrative1y{ww}, devrative2x{ww}, devrative2y{ww});
        sumResidual = sum(residual{ww})
        if count{ww} <= 6
            'less than 6 corresponding points'
            break;
        end
        sigma = 1;
        precision = zeros(3); precision(1,1) = 1/sigma^2; precision(2,2) = 1/sigma^2; precision(3,3) = 1/sigma^2;
        mean2 = 0;
        [weight{ww}, covariance] = computeWeightAndScale(residual{ww}, precision, mean2);
        %weight{ww} = ones(size(weight{ww}));
%         precision = covariance^-1;
        
        ll = computeCompleteDataLogLikelihood(residual{ww}, precision);
        ll = -ll;
        diffll = ll - likilihoodLast
        if diffll > 0
            'likilihood decrease'
            xpose = xLastTime;
            T = xpose2T(xpose);
            break;
        end
        iteration = iteration +1
        likilihoodLast = ll;
        lsList{ww} = zeros([length(residual{ww}),6],'single');
        for ii = 1 : length(residual{ww})
%            Jw = zeros(2,6);
            yy = residualCorres{ww}(ii,1);
            xx = residualCorres{ww}(ii,2);
            pointTmp = reshape(pointcloud1{ww}(yy,xx,:), [1,3]);
%             jz = subs(JZ2, variable, [pointTmp(1), pointTmp(2), pointTmp(3), fx_, fy_, cx_, cy_, depth2{ww}(yy,xx), xpose(1), xpose(2),xpose(3),xpose(4),xpose(5),xpose(6)]);
            [Jw, Jz] = computeJacobianOfProjectionAndTransformation(pointTmp, depth2{ww}(yy,xx), xpose, JZtmp, variable, devrative2x{ww}(yy,xx), devrative2y{ww}(yy,xx));
%             J = [devrative1x{ww}(yy,xx), devrative1y{ww}(yy,xx)] * Jw - Jz;
            J =   Jz;
            lsList{ww}(ii,:) = J;
            if sum(isnan(J))
                'nan in Jacobian'
                continue;
            end
            weightedJ = J' .* weight{ww}(ii)';
            A = A + weightedJ*J;
            b = b - weightedJ*residual{ww}(ii,1);
        end
%         break;
        x = A^(-1)*b;
        ifHaveNewx = true;
% %         xRecord{ww}(iteration) = x;
    end
      
end
T = xpose2T(xLastTime);
T = T^(-1);
xpose = T2xpose(T);