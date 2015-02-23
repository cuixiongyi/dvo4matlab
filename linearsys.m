syms z1 z2 z3 z4 u m l g M
dxdt = [z2; 
        (u+m*l*sin(z3)*z4^2-m*g*cos(z3)*sin(z3)) / (M+m-m*cos(z3)^2);
        z4;
        (u*cos(z3)-(M+m)*g*sin(z3)+m*l*cos(z3)*sin(z3)*z4^2) / (m*l*cos(z3)^2-(M+m)*l)];
    
    jacobian(dxdt, [z1, z2, z3, z4])
 
 