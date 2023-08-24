syms alpha a theta d real
T=despZ(d)*rotZ(theta)*despX(a)*rotX(alpha);

di=[0.089159 0.1093 -0.1093 0.1093 0.09475 0];
ai=[0 0 0.425 0.392 0 0];
alphai=[0 pi/2 0 0 -pi/2 pi/2];
syms q1 q2 q3 q4 q5 q6 real
qi=[q1 q2 q3 q4 q5 q6];

T01=subs(T,{a,alpha,d,theta},{ai(1),alphai(1),di(1),qi(1)});
T12=subs(T,{a,alpha,d,theta},{ai(2),alphai(2),di(2),qi(2)});
T23=subs(T,{a,alpha,d,theta},{ai(3),alphai(3),di(3),qi(3)});
T34=subs(T,{a,alpha,d,theta},{ai(4),alphai(4),di(4),qi(4)});
T45=subs(T,{a,alpha,d,theta},{ai(5),alphai(5),di(5),qi(5)});
T56=subs(T,{a,alpha,d,theta},{ai(6),alphai(6),di(6),qi(6)});

%T01 computed before.
T02=simplify(T01*T12);
T03=simplify(T02*T23);
T04=simplify(T03*T34);
T05=simplify(T04*T45);
T06=simplify(T05*T56);

% UR5 symbolic Jacobian
%joint vectors (zi)
zL=[0 0 1]';
z0=zL;
z1=T01(1:3,1:3)*zL;
z2=T02(1:3,1:3)*zL;
z3=T03(1:3,1:3)*zL;
z4=T04(1:3,1:3)*zL;
z5=T05(1:3,1:3)*zL;

%position vectors (pi); they are the position part of the homogeneus
%transformation matrices (position of the origin of the reference frame)
p0=[0 0 0]';
p1=T01(1:3,4);
p2=T02(1:3,4);
p3=T03(1:3,4);
p4=T04(1:3,4);
p5=T05(1:3,4);
p6=T06(1:3,4);


%Columns according to the algorithm (J=[JP;JO]); we are left with the three
%first elements of each pi vector
J1=simplify([cross(z0,(p6(1:3)-p0(1:3)));z0]);
J2=simplify([cross(z1,(p6(1:3)-p1(1:3)));z1]);
J3=simplify([cross(z2,(p6(1:3)-p2(1:3)));z2]);
J4=simplify([cross(z3,(p6(1:3)-p3(1:3)));z3]);
J5=simplify([cross(z4,(p6(1:3)-p4(1:3)));z4]);
J6=simplify([cross(z5,(p6(1:3)-p5(1:3)));z5]);

JPO=simplify([J1 J2 J3 J4 J5 J6]);


