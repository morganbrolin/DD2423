% function E = compute_E_matrix( points1, points2, K1, K2 );
%
% Method:   Calculate the E matrix between two views from
%           point correspondences: points2^T * E * points1 = 0
%           we use the normalize 8-point algorithm and 
%           enforce the constraint that the three singular 
%           values are: a,a,0. The data will be normalized here. 
%           Finally we will check how good the epipolar constraints:
%           points2^T * E * points1 = 0 are fullfilled.
% 
%           Requires that the number of cameras is C=2.
% 
% Input:    points2d is a 3xNxC array storing the image points.
%
%           K is a 3x3xC array storing the internal calibration matrix for
%           each camera.
%
% Output:   E is a 3x3 matrix with the singular values (a,a,0).

function E = compute_E_matrix( points2d, K )

%normalize the points
pa = (K(:,:,1)) \  points2d(:,:,1);
pb = (K(:,:,2)) \  points2d(:,:,2);

Na = compute_normalization_matrices(pa);
Nb = compute_normalization_matrices(pa);


pa = Na* pa;
pb = Nb* pb;

% Construct W from pa & pb (assuming equal length)
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

E = Nb'*F*Na;

[U,S,V] = svd(E);
s = (S(1,1)+S(2,2))/2;
S = diag([s,s,0]);
E=U*S*V';
% Test if if epipolar constraint holds
epipolar_constraint = diag(pb' * E * pa);
enpolar = pb(:,1)'*E*pa(:,1)
sum_epipolar = sum(sum(abs(epipolar_constraint)))

end