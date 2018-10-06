% Test script for exercise 1

clear all                   % Remove all old variables
close all                   % Close all figures
clc                         % Clear the command window
addpath( genpath( '../' ) );% Add paths to all subdirectories of the parent directory

REFERENCE_VIEW      = 3;
CAMERAS             = 3;

load( '../debug/part1/points2d.mat'     );
load( '../debug/part1/norm_mat.mat'     );
load( '../debug/part1/homographies.mat' );

%% Test the function det_homographies

fprintf('--------------------\n')
fprintf(' HOMOGRAPHIES ERROR \n')
fprintf('--------------------\n')

homographies_test = zeros(3,3,CAMERAS); 

for c=1:CAMERAS
    points_ref = points2d(:,:,REFERENCE_VIEW);
    points_c   = points2d(:,:,c);
    
    homographies_test(:,:,c) = compute_homography( points_ref, points_c );
    
    homographies(:,:,c)      = fix_homogeneous_scale( homographies(:,:,c)      );
    homographies_test(:,:,c) = fix_homogeneous_scale( homographies_test(:,:,c) );
end

homographies_error = homographies - homographies_test


%% Test the function get_normalization_matrices 

fprintf('----------------------------\n')
fprintf(' NORMALIZATION MATRIX ERROR \n')
fprintf('----------------------------\n')

norm_mat_test = compute_normalization_matrices( points2d );

check1 = norm_mat;
check2 = norm_mat_test;

points2dnorm = zeros(size(points2d,1), size(points2d,2), CAMERAS);

for c=1:CAMERAS
    norm_mat(:,:,c)      = fix_homogeneous_scale( norm_mat(:,:,c)      );
    norm_mat_test(:,:,c) = fix_homogeneous_scale( norm_mat_test(:,:,c) );
    
    points2dnorm(:,:,c) = check2(:,:,c) * points2d(:,:,c);
end

norm_mat_error = norm_mat - norm_mat_test

norm_check = compute_normalization_matrices(points2dnorm);

for c = 1 : CAMERAS
%     norm_test(:,:,c)
    difference = norm_check(:,:,c) - eye(3)
end





