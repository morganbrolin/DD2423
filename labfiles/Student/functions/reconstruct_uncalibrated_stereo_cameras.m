% function [cams, cam_centers] = reconstruct_uncalibrated_stereo_cameras(F); 
%
% Method: Calculate the first and second uncalibrated camera matrix
%         from the F-matrix. 
% 
% Input:  F - Fundamental matrix with the last singular value 0 
%
% Output:   cams is a 3x4x2 array, where cams(:,:,1) is the first and 
%           cams(:,:,2) is the second camera matrix.
%
%           cam_centers is a 4x2 array, where (:,1) is the first and (:,2) 
%           the second camera center.

function [cams, cam_centers] = reconstruct_uncalibrated_stereo_cameras( F )
cams(:,:,1) = [eye(3), zeros(3,1)];
S = [0 -1 1
    1 0 -1
    -1 1 0];
%h = F'*zeros(3,1);
h = null(F');
cams(:,:,2) = [S*F, h];
[U1, S1, V1] = svd(cams(:,:,1));
[U2, S2, V2] = svd(cams(:,:,2));
cam_centers = [V1(:,end),V2(:,end)];
%------------------------------
% TODO: FILL IN THIS PART
