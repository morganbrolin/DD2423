
% Method:   Generate one image out of multiple images. All images are from
%           a camera with the same (!) center of projection. All the images 
%           are registered to one reference view.

clear all                   % Remove all old variables
close all                   % Close all figures
clc                         % Clear the command window
addpath( genpath( '../' ) );% Add paths to all subdirectories of the parent directory

LOAD_DATA           = false;
REFERENCE_VIEW      = 3;
CAMERAS             = 3;
image_names_file    = '../images/names_images_kthsmall.txt';
name_panorama       = '../images/panorama_image.jpg';
points2d_file       = '../data/data_kth.mat';

[images, name_loaded_images] = load_images_grey( image_names_file, CAMERAS );

% Load the clicked points if they have been saved,
% or click some new points:
if LOAD_DATA
    load( points2d_file );
else
    points2d = click_multi_view( images ); %, C, data, 0 ); % for clicking and displaying data
    save( points2d_file, 'points2d' );
end


%% Compute homographies
% Determine all homographies to a reference view. We have:
% point in REFERENCE_VIEW = homographies(:,:,c) * point in image c.
% Remember, you have to set homographies{REFERENCE_VIEW} as well.
homographies = zeros(3,3,CAMERAS);
norm_homographies = zeros(3,3,CAMERAS);
norm_points2d = zeros(size(points2d,1),size(points2d,2),size(points2d,3));

% normalize points
norm_mat = compute_normalization_matrices( points2d );
for c = 1:CAMERAS
    norm_points2d(:,:,c) = norm_mat(:,:,c) * points2d(:,:,c);
end

for c = 1:CAMERAS
    points_ref = points2d(:,:,REFERENCE_VIEW);
    points_c   = points2d(:,:,c);
    
    norm_points_ref = norm_points2d(:,:,REFERENCE_VIEW);
    norm_points_c   = norm_points2d(:,:,c);

    norm_homographies(:,:,c) = compute_homography( norm_points_ref, norm_points_c );
    
    [error_mean error_max] = check_error_homographies( ...
      norm_homographies(:,:,c), points2d(:,:,c), points2d(:,:,REFERENCE_VIEW) );
  
    % computer real H
    homographies(:,:,c) = inv(norm_mat(:,:,REFERENCE_VIEW)) * norm_homographies(:,:,c) * norm_mat(:,:,c);
 
    [error_mean error_max] = check_error_homographies( ...
      norm_homographies(:,:,c), points2d(:,:,c), points2d(:,:,REFERENCE_VIEW) );
    
    fprintf( 'Between view %d and ref. view; ', c );
    fprintf( 'average error: %5.2f; maximum error: %5.2f \n', error_mean, error_max );
end
%% Generate, draw and save panorama

panorama_image = generate_panorama_alt( images, homographies );

figure;  
show_image_grey( panorama_image );
save_image_grey( name_panorama, panorama_image );
