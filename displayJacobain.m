ww = 1;

channel =3;


imJa = zeros(size(depth1{ww}),'single');
JTmp = Jlist(:,channel);
% [mink, iiT] = min(JTmp);
% JTmp(ii) = median(JTmp);
% JTmp = JTmp-(min(weightedJ(channel,:)));
% JTmp = lsList{ww}(:,1);
% JTmp = JTmp*200;
for ii = 1 : length(Jlist)
yy = residualCorres{ww}(ii,1);
xx = residualCorres{ww}(ii,2);    
   imJa(yy,xx) =  JTmp(ii);
   
end
imshow(imJa, [-3, 2]);
% sum(isnan(lsList{4}))