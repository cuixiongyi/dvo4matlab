
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

xRecord = cell(1,maxLevel);
xLastTime = x;
for ww = maxLevel : -1 : 1
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
%         lsList{ww} = zeros([length(residual{ww}),6],'single');
%         [A, b, lsList{ww}] = computeJacbianLoop(residual{ww}, residualCorres{ww}, pointcloud1{ww}, xpose, JZtmp,variable,devrative2x{ww},devrative2y{ww},weight{ww});
        tic
        testlen = 100;
%         Jlist = mexComputeJacobian(residual{ww}, residualCorres{ww}, pointcloud1{ww}(:,:,1),pointcloud1{ww}(:,:,2),pointcloud1{ww}(:,:,3),xpose, devrative2x{ww},devrative2y{ww},1,[fx_,fy_,cx_,cy_]);
        Jlist = mexComputeJacobian(residual{ww}(1:testlen,:), residualCorres{ww}(1:testlen,:), pointcloud1{ww}(:,:,1),pointcloud1{ww}(:,:,2),pointcloud1{ww}(:,:,3),xpose, devrative2x{ww},devrative2y{ww},1,[fx_,fy_,cx_,cy_]);
        for ii = 1 : length(1:100)
%            Jw = zeros(2,6);
            yy = residualCorres{ww}(ii,1);
            xx = residualCorres{ww}(ii,2);
            pointTmp = reshape(pointcloud1{ww}(yy,xx,:), [1,3]);
%             jz = subs(JZ2, variable, [pointTmp(1), pointTmp(2), pointTmp(3), fx_, fy_, cx_, cy_, depth2{ww}(yy,xx), xpose(1), xpose(2),xpose(3),xpose(4),xpose(5),xpose(6)]);
            [Jw, Jz] = computeJacobianOfProjectionAndTransformation(pointTmp, xpose, JZtmp, variable, devrative2x{ww}(yy,xx), devrative2y{ww}(yy,xx));
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
weightedJ = zeros(size(Jlist'));
weightedb = zeros(size(Jlist'));
newJlist = zeros(size(Jlist));
count2 = 0;
for ii = 1 : length(Jlist)
    if sum(isnan(Jlist(ii,:)),2) > 0
        continue;
    end
    count2 = count2 +1;
    newJlist(count2,:) = Jlist(ii,:);
    weightedJ(:,count2) = Jlist(ii,:)'.*weight{ww}(ii);
    weightedb(:,count2) = Jlist(ii,:)'.*weight{ww}(ii).*residual{ww}(ii,1);
end
newJlist = newJlist(1:count2,:);
weightedJ = weightedJ(:,1:count2);
weightedb = weightedb(:,1:count2);
        toc
        
%         break;
        A = weightedJ*newJlist;
        b = -sum(weightedb, 2);
        x = A^(-1)*b;
        ifHaveNewx = true;
% %         xRecord{ww}(iteration) = x;
    end
      
end
T = xpose2T(xLastTime);
T = T^(-1);
xpose = T2xpose(T);
