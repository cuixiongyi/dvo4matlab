ww = 4;

channel =3;


imJa = zeros(size(depth1{ww}),'int8');
JTmp = lsList{ww}(:,channel);
% [mink, iiT] = min(JTmp);
JTmp(ii) = median(JTmp);
JTmp = JTmp-(min(lsList{ww}(:,channel)));
JTmp = JTmp / max(JTmp)*250;
for ii = 1 : length(residual{ww})
yy = residualCorres{ww}(ii,1);
xx = residualCorres{ww}(ii,2);    
   imJa(yy,xx) =  JTmp(ii);
   
end
imshow(imJa);
% sum(isnan(lsList{4}))