function [A, b, lsList] = computeJacbianLoop(residual, residualCorres, pointcloud1, xpose, JZtmp,variable,devrative2x,devrative2y,weight)
A = zeros(6);
b = zeros(6,1);
yy = residualCorres(:,1);
xx = residualCorres(:,2);
lsList = zeros([length(residual),6],'single');
tic
for ii = 1 : length(residual)
            
            pointTmp = reshape(pointcloud1(yy(ii),xx(ii),:), [1,3]);
%             jz = subs(JZ2, variable, [pointTmp(1), pointTmp(2), pointTmp(3), fx_, fy_, cx_, cy_, depth2{ww}(yy,xx), xpose(1), xpose(2),xpose(3),xpose(4),xpose(5),xpose(6)]);
            [Jw, Jz] = computeJacobianOfProjectionAndTransformation(pointTmp, xpose, JZtmp, variable, devrative2x(yy(ii),xx(ii)), devrative2y(yy(ii),xx(ii)));
%             J = [devrative2x(yy(ii),xx(ii)), devrative2y(yy(ii),xx(ii))] * Jw - Jz;
            J =   Jz;
            lsList(ii,:) = J;
            if sum(isnan(J))
                'nan in Jacobian'
                continue;
            end
            
            weightedJ = J' .* weight(ii)';
            A = A + weightedJ*J;
            b = b - weightedJ*residual(ii,1);

end
toc
'parfor finished'
end