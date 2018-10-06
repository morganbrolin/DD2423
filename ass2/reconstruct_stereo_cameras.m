% function [cams, cam_centers] = reconstruct_stereo_cameras(E, K1, K2, data); 
%
% Method:   Calculate the first and second camera matrix. 
%           The second camera matrix is unique up to scale. 
%           The essential matrix and 
%           the internal camera matrices are known. Furthermore one 
%           point is needed in order solve the ambiguity in the 
%           second camera matrix.
%
%           Requires that the number of cameras is C=2.
%
% Input:    E is a 3x3 essential matrix with the singular values (a,a,0).
%
%           K is a 3x3xC array storing the internal calibration matrix for
%           each camera.
%
%           points2d is a 3xC matrix, storing an image point for each camera.
%
% Output:   cams is a 3x4x2 array, where cams(:,:,1) is the first and 
%           cams(:,:,2) is the second camera matrix.
%
%           cam_centers is a 4x2 array, where (:,1) is the first and (:,2) 
%           the second camera center.
%

function [cams, cam_centers] = reconstruct_stereo_cameras( E, K, points2d )
% decompose e
[U,S,V] = svd(E);
% extract translation t
t = V(:,end);
% calculate rotation matrices R1 & R2 with W
W = [
    0 -1  0;
    1  0  0;
    0  0  1;
    ];
R1 = U*W*V';
R2 = U*W'*V';
% determine if R1 & R2 also mirrors the camera and scale with -1 if they do
%fipped to make it right
if det(R1) > 0
    R1 = R1 * -1;
end
if det(R2) > 0
    R2 = R2 * -1;
end
% compute Mbn and try to reconstruct 3d point for each
Mb1 = K(:,:,2)*R1*[eye(3) t];
Mb2 = K(:,:,2)*R1*[eye(3) -t];
Mb3 = K(:,:,2)*R2*[eye(3) t];
Mb4 = K(:,:,2)*R2*[eye(3) -t];
Ma = K(:,:,1) * [eye(3) [0;0;0]];

p1 = reconstruct_point_cloud(cat(3,Mb1,Ma),points2d);
p2 = reconstruct_point_cloud(cat(3,Mb2,Ma),points2d);
p3 = reconstruct_point_cloud(cat(3,Mb3,Ma),points2d);
p4 = reconstruct_point_cloud(cat(3,Mb4,Ma),points2d);
% loop over the 4 possibilities untill a case where the point is in front
% of both cameras
Mb = cat(3,Mb1,Mb2,Mb3,Mb4);
p = [p1 p2 p3 p4];
R = cat(3,R1,R1,R2,R2);
T = [t -t t -t];
cams = zeros(3,4,2);
for i = 1 : 4
    pb = R(:,:,i)*[eye(3) T(:,i)]*p(:,i);
%     c1 = p(3,i) > 0
%     c2 = pb(3) > 0
    %flipped > to < to make it right
    if p(3,i) < 0 && pb(3) > 0
%     if i == 3 % test, remove later. 3 seems 2 be correct 4 test case
%         fprintf('motherclucker\n')
        cams(:,:,2) = Mb(:,:,i);
        % Verify the rotation and translation
        Etest = R(:,:,i) * [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
        E - Etest;
        break;
    end
end
% set camera1 to K(I|0)
cams(:,:,1) = Ma;
% set camera centers
cam_centers(:,1) = [0;0;0;1];
% fliped -t to t to get it right
cam_centers(:,2) = [t(1);t(2);t(3);1];
end
