% function F = compute_F_matrix(points1, points2);
%
% Method:   Calculate the F matrix between two views from
%           point correspondences: points2^T * F * points1 = 0
%           We use the normalize 8-point algorithm and 
%           enforce the constraint that the three singular 
%           values are: a,b,0. The data will be normalized here. 
%           Finally we will check how good the epipolar constraints:
%           points2^T * F * points1 = 0 are fullfilled.
% 
%           Requires that the number of cameras is C=2.
% 
% Input:    points2d is a 3xNxC array storing the image points.
%
% Output:   F is a 3x3 matrix where the last singular value is zero.

function F = compute_F_matrix( points2d )

%normalize the points
pa =  points2d(:,:,1);
pb =  points2d(:,:,2);

Na = compute_normalization_matrices(pa);
Nb = compute_normalization_matrices(pa);


pa = Na* pa;
pb = Nb* pb;



W = zeros(size(points2d,2), 9);
for i = 1 : size(points2d,2)
    xb=pb(1,i);xa = pa(1,i);ya=pa(2,i);yb=pb(2,i);
    W(i,:) = [xb*xa,xb*ya,xb,yb*xa, ...
       yb*ya, yb, xa, ya, 1];
   
end
% Get h from svd
[U, S, V] = svd(W);
h = V(:,end);
% Structure the vector h as essential matris E

F = [h(1), h(2), h(3); h(4), h(5), h(6); h(7), h(8), h(9)];

%la till detta so fixade
F = Nb'*F*Na;

%constrain to rank2

[U, S, V] = svd(F);
S(end) = 0;
F = U*S*V';
jacobi = F
% Test if if epipolar constraint holds
epipolar_constraint = diag(pb' * F * pa);
enpolar = pb(:,1)'*F*pa(:,1)
sum_epipolar = sum(sum(abs(epipolar_constraint)))
end
