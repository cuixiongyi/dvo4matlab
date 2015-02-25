syms X1 Y1 Z1 v1 v2 v3 w1 w2 w3 Xihat Xi fxs fys cxs cys depth2dx depth2dy  g complex

Xi = [v1 v2 v3 w1 w2 w3];
Xihat = [0,-Xi(6), Xi(5);
         Xi(6),0, -Xi(4);
         -Xi(5), Xi(4), 0];
R = RodriguesSyms([w1,w2,w3]);
g(1:3,1:3) = R;
g(1:3,4)=[v1,v2,v3]';
%g(1:4,1:3) = 0;
g(4,4) = 1; %g(3,3) = 1; g(2,2) = 1; g(1,1) = 1;

Z2 = (g*[X1,Y1,Z1, 1]');
uprojected = Z2(1)*fxs/Z2(3)+cxs;
vprojected = Z2(2)*fys/Z2(3)+cys;

r2 = depth2dx*uprojected+depth2dy*vprojected-Z2(3);
% r2 = -Z2(3);
JZ2 = jacobian(r2, [v1 v2 v3 w1 w2 w3]);
JZ_full = JZ2;
%% cosine approximation cos = 1-theta^2/2
JZ_full2 = subs(JZ_full, cos((w1^2 + w2^2 + w3^2)^(1/2)), 1-(((w1^2 + w2^2 + w3^2)^(1/2))^2)/2);
JZ_full3 = subs(JZ_full2, sin((w1^2 + w2^2 + w3^2)^(1/2)), (w1^2 + w2^2 + w3^2)^(1/2));
JZtmp = simplify(JZ_full3,'Steps', 100);
pretty(JZtmp)
% JZtmp= JZ2;
%% cos = 1 sin = 0
% JZ_full2 = subs(JZ2, cos((w1^2 + w2^2 + w3^2)^(1/2)), 1);
% JZ_full3 = subs(JZ_full2, sin((w1^2 + w2^2 + w3^2)^(1/2)), 0);


