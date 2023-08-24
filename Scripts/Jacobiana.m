%% Fórmulas para obtener las transformadas de desplazamiento y rotación


% Matriz de desplazamiento en X
function D=despX(p)

D=[1,0,0,p;0,1,0,0;0,0,1,0;0,0,0,1];

end

% Matriz de desplazamiento en Y
function D=despY(p)

D=[1,0,0,0;0,1,0,p;0,0,1,0;0,0,0,1];

end

% Matriz de desplazamiento en Z
function D=despZ(p)

D=[1,0,0,0;0,1,0,0;0,0,1,p;0,0,0,1];
end

% Matriz de rotación en X
function RX=rotX(theta)

s=sin(theta); c=cos(theta);
if isnumeric(theta)
    if abs(c)<1e-10
        c=0;
    end
    if abs(s)<1e-10
        s=0;
    end
end 
RX=[1,0,0,0;0,c,-s,0;0,s,c,0;0,0,0,1];
end

% Matriz de rotación en Y
function RY=rotY(theta)

s=sin(theta); c=cos(theta);
if isnumeric(theta)
    if abs(c)<1e-10
        c=0;
    end
    if abs(s)<1e-10
         s=0;
    end
end  

RY=[c,0,s,0;0,1,0,0;-s,0,c,0;0,0,0,1];
end
% Matriz de rotación en Z
function RZ=rotZ(theta)

s=sin(theta); c=cos(theta);
if isnumeric(theta)
    if abs(c)<1e-10
        c=0;
    end
    if abs(s)<1e-10
         s=0;
    end
end 

RZ=[c,-s,0,0;s,c,0,0;0,0,1,0;0,0,0,1];
end

%% Parametros DH del UR5

syms alpha a theta d real
T=despZ(d)*rotZ(theta)*despX(a)*rotX(alpha);
di=[0.290 0 0 0.302 0 0.072];
ai=[0 0.270 0.070 0 0 0];
alphai=[-pi/2 0 -pi/2 pi/2 -pi/2 0];
syms q1 q2 q3 q4 q5 q6 real
qi=[q1 q2 q3 q4 q5 q6];

% Transformada homogenea por pasos
T01=subs(T,{a,alpha,d,theta},{ai(1),alphai(1),di(1),qi(1)});
T12=subs(T,{a,alpha,d,theta},{ai(2),alphai(2),di(2),qi(2)});
T23=subs(T,{a,alpha,d,theta},{ai(3),alphai(3),di(3),qi(3)});
T34=subs(T,{a,alpha,d,theta},{ai(4),alphai(4),di(4),qi(4)});
T45=subs(T,{a,alpha,d,theta},{ai(5),alphai(5),di(5),qi(5)});
T56=subs(T,{a,alpha,d,theta},{ai(6),alphai(6),di(6),qi(6)});

% T01 ya está hecha.
T02=simplify(T01*T12);
T03=simplify(T02*T23);
T04=simplify(T03*T34);
T05=simplify(T04*T45);
T06=simplify(T05*T56);

% Jacobiana del UR5
% Vectores de las articulaciones (zi)
zL=[0 0 1]';
z0=zL;
z1=T01(1:3,1:3)*zL;
z2=T02(1:3,1:3)*zL;
z3=T03(1:3,1:3)*zL;
z4=T04(1:3,1:3)*zL;
z5=T05(1:3,1:3)*zL;

% Vectores de posición (pi); Son la parte de la transformada homogenea que
% corresponde con el desplazamiento
p0=[0 0 0]';
p1=T01(1:3,4);
p2=T02(1:3,4);
p3=T03(1:3,4);
p4=T04(1:3,4);
p5=T05(1:3,4);
p6=T06(1:3,4);

% Algoritmo (J
























