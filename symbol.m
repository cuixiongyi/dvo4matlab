syms x y z v1 v2 v3 w1 w2 w3 fx_ fy_ cx_ cy_ dx dy  g  w1sq w2sq w3sq w1w2 w1w3 w2w3 

w1sq = w1*w1;
w2sq = w2*w2;
w3sq = w3*w3;
w1w3 = w1*w3;
w1w2 = w1*w2;
w2w3 = w2*w3;


g(1:3,1:3) = R;
g(1:3,4)=[v1,v2,v3]';
%g(1:4,1:3) = 0;
g(4,4) = 1; %g(3,3) = 1; g(2,2) = 1; g(1,1) = 1;

Z2 = (g*[x,y,z, 1]');
uprojected = Z2(1)*fx_/Z2(3)+cx_;
vprojected = Z2(2)*fy_/Z2(3)+cy_;

r2 = dx*uprojected+dy*vprojected-Z2(3);
% r2 = -Z2(3);
JZ2 = jacobian(r2, [v1 v2 v3 w1 w2 w3]);

JZ_full = JZ2;
%% cosine approximation cos = 1-theta^2/2
JZ_full2 = subs(JZ_full, cos((w1^2 + w2^2 + w3^2)^(1/2)), 1-(((w1^2 + w2^2 + w3^2)^(1/2))^2)/2);
JZ_full3 = subs(JZ_full2, sin((w1^2 + w2^2 + w3^2)^(1/2)), (w1^2 + w2^2 + w3^2)^(1/2));
sj = simplify(JZ_full3);
sj2 = subs(sj, w1*w1, w1sq);
sj2 = subs(sj2, w2*w2, w2sq);
sj2 = subs(sj2, w3*w3, w3sq);
sj2 = subs(sj2, w1*w2, w1w2);
sj2 = subs(sj2, w1*w3, w1w3);
sj2 = subs(sj2, w2*w3, w2w3);
syms e1 e2 e3 e4 e5 e6 e7 e8 e9 e10 e11 e12 e13 e14 e15 e16 e17 e18 e19 real
sj2 = subs(sj2, (2 * v3 - z*(w1sq + w2sq - 2) - x*(2 * w2 - w1w3) + y*(2 * w1 + w2w3)), e1);