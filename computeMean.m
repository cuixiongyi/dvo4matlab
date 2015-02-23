function [mean2] = computeMean(residual, weight)
    
    mean2 = zeros([1,3]);
    mean2(1) = sum(residual(:,1).*weight) / sum(weight);
    mean2(2) = sum(residual(:,2).*weight) / sum(weight);
    mean2(3) = sum(residual(:,3).*weight) / sum(weight);
end