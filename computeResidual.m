function [count, residual, residualCorres, projectedPointcloud] = computeResidual(T, depth1, depth2, pointcloud1, pointcloud2, devrative1x, devrative1y, devrative2x, devrative2y)
    global fx_ fy_ cx_ cy_;
%     global wcur wref;
wzd = 0;
    wcur_zd = wzd; wref_zd = -wzd;

    %% Project reference image into current
    [height, width] = size(depth1);
    K = [fx_, 0, cx_; 0, fy_, cy_; 0,0,1];
    wcur =[1.0, wcur_zd * fx_, wcur_zd * fy_];
    wref =[-1.0, wref_zd * fx_, wref_zd * fy_];
    T_2 = T(1:3,:);
%     project1 = zeros([size(depth1),2], 'single');
%     project2 = zeros([size(depth1),2], 'single');
    residual = zeros([ size(depth1,1)*size(depth1,2),3], 'single');
    residualCorres = zeros([ size(depth1,1)*size(depth1,2),2], 'uint16');
    projectedPointcloud = zeros([ size(depth1),3], 'single');
    count = 0;
    
    for ii = 2 : height-1
        for jj = 2 : width-1
            if isnan(depth1(ii,jj)) || isnan(depth2(ii,jj)) %|| isnan(depth2(ii+1,jj)) || isnan(depth2(ii,jj+1)) || isnan(depth2(ii+1,jj+1)) 
                continue;
            end
            % 4*1 vector
            
            projectedPointcloud(ii,jj,:) = T(1:3,:) * [reshape(pointcloud1(ii,jj,:),[3,1]);1];
        end
    end
    devrativeProjectx = zeros(size(depth1), 'single');
    devrativeProjecty = zeros(size(depth1), 'single');
    for ii = 2 : height-1
            devrativeProjecty(ii,:) = (projectedPointcloud(ii+1,:,3) - projectedPointcloud(ii-1,:,3)) /2;
    end
    for ii = 2 : width-1
            devrativeProjectx(:,ii) = (projectedPointcloud(:,ii+1,3) - projectedPointcloud(:,ii-1,3)) /2;
    end
    for ii = 2 : height-1
        for jj = 2 : width-1
            
            if isnan(depth1(ii,jj)) || isnan(projectedPointcloud(ii,jj,3)) || isnan(devrativeProjecty(ii,jj)) || isnan(devrativeProjectx(ii,jj))%|| isnan(depth2(ii+1,jj)) || isnan(depth2(ii,jj+1)) || isnan(depth2(ii+1,jj+1)) 
                continue;
            end
%             projected2 = T * [reshape(pointcloud1(ii,jj,:),[3,1]);1];
%             projected2(1) =  projected2(1)*fx_/projected2(3) + cx_;
%             projected2(2) =  projected2(2)*fy_/projected2(3) + cy_;
            projected2x = projectedPointcloud(ii,jj,1)*fx_/projectedPointcloud(ii,jj,3)+ cx_;
            projected2y = projectedPointcloud(ii,jj,2)*fy_/projectedPointcloud(ii,jj,3) + cy_;
            if ~(projected2x > 0 && projected2x < width && projected2y > 0 && projected2y < height)
                continue;
            end
            x0 = floor(projected2x); y0 = floor(projected2y);
            if x0 == 0 || y0 == 0
                continue;
            end
            x1w = projected2x - x0;
            x0w = 1.0 - x1w;
            y1w = projected2y - y0;
            y0w = 1.0 - y1w;
            % z,  zdx, zdy
            x0y0 = [depth2(y0,x0), devrative2x(y0,x0), devrative2y(y0,x0)];
            x1y0 = [depth2(y0,x0+1), devrative2x(y0,x0+1), devrative2y(y0,x0+1)];
            x0y1 = [depth2(y0+1,x0), devrative2x(y0+1,x0), devrative2y(y0+1,x0)];
            x1y1 = [depth2(y0+1,x0+1), devrative2x(y0+1,x0+1), devrative2y(y0+1,x0+1)];
            interpolated = (x0y0 * x0w + x1y0 * x1w) * y0w + (x0y1 * x0w + x1y1 * x1w) * y1w;
            
            %TODO test things~~~~~~~~~~
%             point1 = reshape(pointcloud1(ii,jj,:),[3,1]);
%             point2 = reshape(pointcloud2(ii,jj,:),[3,1]);
            reference = [projectedPointcloud(ii,jj,3), devrativeProjectx(ii,jj), devrativeProjecty(ii,jj)];
            residual(count+1,:) = wcur.*interpolated + wref.*reference;
            residualCorres(count+1,:) = [ii,jj];
            if sum(isnan(residual(count+1,:))) > 0 || sum(isnan(devrative1x(ii,jj))) || sum(isnan(devrative1y(ii,jj)))
                continue;
            end
%             if abs(residual(count+1,1)) > 5
%                 continue;
%             end
            count = count +1;
        end
    end
    residual = residual(1:count,:);
    residualCorres = residualCorres(1:count,:);
end