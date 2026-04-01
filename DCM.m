function [C_be] = DCM(X)
%Find Earth to body Direction Cosine Matrix (DCM)

% Extract variables
q0=X(7);
q1=X(8);
q2=X(9);
q3=X(10);

% Compute first row of DCM
l1 = (q0^2)+(q1^2)-(q2^2)-(q3^2);
l2 = 2*((q1*q2)+(q0*q3));
l3 = 2*((q1*q3)-(q0*q2));
row1 = [l1 l2 l3];

% Compute first row of DCM
m1 = 2*((q1*q2)-(q0*q3));
m2 = (q0^2)-(q1^2)+(q2^2)-(q3^2);
m3 = 2*((q2*q3)-(q0*q1));
row2 = [m1 m2 m3];

% Compute first row of DCM
n1 = 2*((q0*q2)+(q1*q3));
n3 = (q0^2)-(q1^2)-(q2^2)+(q3^2);
n2 = 2*((q2*q3)-(q0*q1));
row3 = [n1 n2 n3];

% Combine elements into 3x3 matrix
C_be = [row1;row2;row3];
end