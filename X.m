% X(vector) returns the skew-symetric cross product matrix.
% Vector must be a column matrix of length 3.
function [crossmatrix] = X(vector)

if size(vector)~=[3,1]
    error('input must be a vector')
end

crossmatrix = [0          -vector(3,1)     vector(2,1);
               vector(3,1)     0          -vector(1,1);
              -vector(2,1) vector(1,1)         0     ];


end
