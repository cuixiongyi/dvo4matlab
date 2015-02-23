function [weight, covariance] = computeWeightAndScale(residual, precision, mean2)
   % meanTmp = [0,0,0];
    scale = 1.0/(length(residual)-2-1);
    freedom = 5.0;
    weight = zeros([length(residual),1], 'single');
    covariance = zeros(3);
    
    for ii = 1 : length(weight)
        diff = residual(ii,:);
        weight(ii) = (2.0 + freedom) / (freedom + diff * precision * diff');
%         covariance = covariance + weight(ii) * (diff'*diff);
    end
    %a = sum(isnan(weight));
    covariance = covariance *scale;
end