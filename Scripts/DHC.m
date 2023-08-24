function A=DHC(alpha,a,d,theta)

M=[    cos(theta),     -sin(theta)*cos(alpha),     sin(theta)*sin(alpha),      a*cos(theta);
        sin(theta),     cos(alpha)*cos(theta),      -sin(alpha)*cos(theta),     a*sin(theta);
        0,              sin(alpha),                 cos(alpha),                 d;
        0,              0,                          0,                          1];

A=M;



    