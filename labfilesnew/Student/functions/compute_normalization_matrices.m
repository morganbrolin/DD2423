% Method:   compute all normalization matrices.  
%           It is: point_norm = norm_matrix * point. The norm_points 
%           have centroid 0 and average distance = sqrt(2))
% 
%           Let N be the number of points and C the number of cameras.
%
% Input:    points2d is a 3xNxC array. Stores un-normalized homogeneous
%           coordinates for points in 2D. The data may have NaN values.
%        
% Output:   norm_mat is a 3x3xC array. Stores the normalization matrices
%           for all cameras, i.e. norm_mat(:,:,c) is the normalization
%           matrix for camera c.

function norm_mat = compute_normalization_matrices( points2d )
% adding non NaN vectors to compute mean
pc = zeros(3,size(points2d,3));
for  C = 1:size(points2d,3)
    points1 = points2d(:,:,C);
    p = zeros(3,1);
    N = 0;
    for i = 1 : size(points1,2)
        if sum(isnan(points1(:,i))) == 0
            N = N + 1;
            p = p + points1(:,i);
        end
    end
    p = p / N;
    pc(:,C) = p;
end

% computing variance ignoring NaN vectors
dc = zeros(size(points2d,3),1);
for  C = 1:size(points2d,3)
    points1 = points2d(:,:,C);
    p = pc(:,C);
    d = 0;
    N = 0;
    for i = 1 : size(points1,2)
        if sum(isnan(points1(:,i))) == 0
            N = N + 1;
            d = d + norm(points1(:,i) - p, 2);
        end
    end
    d = d / N;
    dc(C) = d;
end

% form normalization matrices from pc and dc
Nc = zeros(3,3,size(points2d,3));
for  C = 1:size(points2d,3)
    Nc(:,:,C) = sqrt(2)/dc(C) * [1,0,-pc(1,C); 0,1,-pc(2,C); 0,0,dc(C)/sqrt(2)];
end

norm_mat = Nc;
end