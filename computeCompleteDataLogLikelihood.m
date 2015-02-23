function [ll] = computeCompleteDataLogLikelihood(residual, precision)
    error_sum = 0.0;
    error_acc = 1.0;
    c = 1;
    for ii = 1 : length(residual)
       error_acc = error_acc * (1.0 + 0.2 * (residual(ii,:)*precision*residual(ii,:)'));
       c = c+1;
       if mod(c, 50) == 0
           error_sum = log(error_acc) + error_sum;
           c = 1;
           error_acc = 1;
       end
    end
    
    ll = 0.5 * length(residual) * log(det(precision)) - 0.5 * (5.0 + 2.0) * error_sum;
end