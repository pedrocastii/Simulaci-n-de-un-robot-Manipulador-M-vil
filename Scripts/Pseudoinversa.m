syms R;
syms L;
syms theta;

A=[R*(cos(theta)/2) R*(cos(theta)/2); R*(sin(theta)/2) R*(sin(theta)/2); R/L -R/L]
B=transpose(A)
C=B*A
D=inv(C)
F=D*B
pseudoinversa=simplify(F)
