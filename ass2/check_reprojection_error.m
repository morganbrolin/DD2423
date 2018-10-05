% function [error_average, error_max] = check_reprojection_error(data, cam, model)
%
% Method:   Evaluates average and maximum error 
%           between the reprojected image points (cam*model) and the 
%           given image points (data), i.e. data = cam * model 
%
%           We define the error as the Euclidean distance in 2D.
%
%           Requires that the number of cameras is C=2.
%           Let N be the number of points.
%
% Input:    points2d is a 3xNxC array, storing all image points.
%
%           cameras is a 3x4xC array, where cams(:,:,1) is the first and 
%           cameras(:,:,2) is the second camera matrix.
%
%           point3d 4xN matrix of all 3d points.
%       
% Output:   
%           The average error (error_average) and maximum error (error_max)
%      

function [error_average, error_max] = check_reprojection_error( points2d, cameras, point3d )
error_average = 0;
% convert 3d points to 2d points for both cameras
pa = cameras(:,:,1) * point3d;
pb = cameras(:,:,2) * point3d;
% convert all points to cartesian coordinates
p1 = homogeneous_to_cartesian(points2d(:,:,1));
p2 = homogeneous_to_cartesian(points2d(:,:,2));
pa = homogeneous_to_cartesian(pa);
pb = homogeneous_to_cartesian(pb);
% calculate difference between points
% p_diff1 = p1 - pa;
% p_diff2 = p2 - pb;
% compute distance of difference vectors
d1 = diag(pdist2(p1',pa'));
d2 = diag(pdist2(p2',pb'));
% extract maximum error
error_max = max(max(d1),max(d2));
% get mean error
error_average = mean([d1' d2']);
end