% Script: uncalibrated_reconstruction.m 
%
% Method: Performs a reconstruction of two 
%         uncalibrated cameras. The Fundametal matrix determines 
%         uniquely both cameras. The point-structure is obtained 
%         by triangulation. The projective reconstruction is 
%         rectified to a metric reconstruction with knowledge 
%         about the 3D object.
%         Finally the result is stored as a VRML model 
% 

clear all                   % Remove all old variables
close all                   % Close all figures
clc                         % Clear the command window
addpath( genpath( '../' ) );% Add paths to all subdirectories of the parent directory

REFERENCE_VIEW      = 1;
CAMERAS             = 2;
image_names_file    = 'names_images_toyhouse.txt';

SYNTHETIC_DATA      = 1;
REAL_DATA_CLICK     = 2;
REAL_DATA_LOAD      = 3;
VERSION             = REAL_DATA_LOAD;

if VERSION == SYNTHETIC_DATA
    points2d_file = '../data/data_sphere.mat';
    points3d_file = '../data/data_sphere_reconstruction.mat';
else
    points2d_file = '../data/data_morg_toyhouse.mat';
end


%% Load the 2d data
%'../data/data_test_toyhouse.mat'; t?rn + morg b?st
%'../data/data_morg_toyhouse.mat'; ny morg grounddata
if VERSION == SYNTHETIC_DATA
    
    load( points2d_file );
    images = cell(CAMERAS,1);
    
elseif VERSION == REAL_DATA_CLICK
    
    [images image_names] = load_images_grey( image_names_file, CAMERAS ); 
    points2d = click_multi_view( images );%, CAMERAS , data, 0); % for clicking and displaying data
    save( points2d_file, 'points2d' );
    
elseif VERSION == REAL_DATA_LOAD
        
    [images,image_names] = load_images_grey( image_names_file, CAMERAS ); 
    load( points2d_file );
    
else
    return
end

%% Do projective reconstruction

F = compute_F_matrix( points2d );

[cameras camera_centers] = reconstruct_uncalibrated_stereo_cameras( F );
camera_centers
points3d = reconstruct_point_cloud( cameras, points2d );

[error_average error_max] = check_reprojection_error( points2d, cameras, points3d );
fprintf( '\n\nThe reprojection error: points2d = cameras * points3d is: \n' );
fprintf( 'Average error: %5.2fpixel; Maximum error: %5.2fpixel \n', error_average, error_max ); 


%% Rectify the projective reconstruction to a metric reconstruction

if VERSION == SYNTHETIC_DATA
    
    load( points3d_file ); % Load points3d_synthetic
    indices = [1 24 25 37 48];
    points3d_ground_truth = points3d_synthetic( :, indices );
    
else 
    % origo h?rn v?nster nere
    %kanten till h?ger
    % kanten l?ngst bort till v?nster
    % kanten n?rmast till v?nster
    % kanten tak till v?nster
    %ogpoint = homogeneous_to_cartesian(points3d(:,1));
    %ogpoint = [ogpoint;0];
    %points3d_ground_truth = [0, 27, 0, 0, 0
    %                         0, 0, 10, 0, 10
    %                         0, 0, 0, 9, 9
    %                         1, 1, 1, 1, 1 ];
    %points3d_ground_truth = points3d_ground_truth  + ogpoint;

    
    %fr?n z niv? 0 -> niv? 9 sen niv? 15 v?nster till h?ger 
    points3d_ground_truth = [0, 0, 27, 0, 0, 27,27
                             10, 0, 0, 10, 0, 0,5
                             0, 0, 0, 9, 9, 9,15
                             1, 1, 1, 1, 1, 1, 1 ];
    indices = [1,2,5,6,7];
    points3d_ground_truth = points3d_ground_truth( :, indices );
end

H = compute_rectification_matrix( points3d(:,indices), points3d_ground_truth );
[error_average_before error_max_before ] = check_reprojection_error( points2d, cameras, points3d );
points3d = H*points3d;
[error_average_after error_max_afgter] = check_reprojection_error( points2d, cameras, points3d );
%camera_centers = H*camera_centers;
%camera_centers = [homogeneous_to_cartesian(H*camera_centers);1,1]

visualize_reconstruction( points3d, camera_centers, ...
    points2d( :, :, REFERENCE_VIEW ), images{REFERENCE_VIEW} )

