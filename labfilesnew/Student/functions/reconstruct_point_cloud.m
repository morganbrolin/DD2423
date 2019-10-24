% function model = reconstruct_point_cloud(cam, data)
%
% Method:   Determines the 3D model points by triangulation
%           of a stereo camera system. We assume that the data 
%           is already normalized 
% 
%           Requires that the number of cameras is C=2.
%           Let N be the number of points.
%
% Input:    points2d is a 3xNxC array, storing all image points.
%
%           cameras is a 3x4xC array, where cameras(:,:,1) is the first and 
%           cameras(:,:,2) is the second camera matrix.
% 
% Output:   points3d 4xN matrix of all 3d points.


function points3d = reconstrut_point_cloud( cameras, points2d )

N = size(points2d,2);
points3d = zeros(4,N);

ma = cameras(:,:,1);
mb = cameras(:,:,2);

for i = 1 : N
    xa = points2d(1,i,1); xb = points2d(1,i,2); ya = points2d(2,i,1); yb = points2d(2,i,2);
    W = [
  [xa ya xb yb] .* [ma(3,1) ma(3,1) mb(3,1) mb(3,1)] - [ma(1,1) ma(2,1) mb(1,1) mb(2,1)];
  [xa ya xb yb] .* [ma(3,2) ma(3,2) mb(3,2) mb(3,2)] - [ma(1,2) ma(2,2) mb(1,2) mb(2,2)];
  [xa ya xb yb] .* [ma(3,3) ma(3,3) mb(3,3) mb(3,3)] - [ma(1,3) ma(2,3) mb(1,3) mb(2,3)];
  [xa ya xb yb] .* [ma(3,4) ma(3,4) mb(3,4) mb(3,4)] - [ma(1,4) ma(2,4) mb(1,4) mb(2,4)];
        ]';
    
    [U,S,V] = svd(W);
    P = V(:,end);
    points3d(:,i) = P;
end
end
