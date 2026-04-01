function out = C_x(A)
% Rotation Matrix about x-axis
row1 = [1, 0, 0];
row2 = [0, cos(A), sin(A)];
row3 = [0, -sin(A), cos(A)];
C_x = [row1; row2; row3];

out = C_x;
end