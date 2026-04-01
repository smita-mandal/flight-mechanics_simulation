function out = C_y(A)
% Rotation Matrix about x-axis
row1 = [cos(A), 0, -sin(A)];
row2 = [0, 1, 0];
row3 = [sin(A), 0, cos(A)];
C_y = [row1; row2; row3];

out = C_y;
end