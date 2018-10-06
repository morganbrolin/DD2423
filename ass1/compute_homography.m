% H = compute_homography(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input:  points1, points2 are of the form (3,n) with 
%         n is the number of points.
%         The points should be normalized for 
%         better performance.
% 
% Output: H 3x3 matrix 
%

function H = compute_homography( points1, points2 )
% get non-NaN points from matrices
p1 = []; p2 = [];
for i = 1 : size(points1,2)
    if sum(isnan(points1(:,i))) < 1 && sum(isnan(points2(:,i))) < 1
        p1 = [p1 points1(:,i)];
        p2 = [p2 points2(:,i)];
    end
end
% size(p1)
% size(p2)
% define Q from points 1 & 2, assuming same size of points 1 & 2
Q = zeros(size(p1,2) * 2, 9);
for i = 1 : size(Q,1) / 2
    % alpha
    Q(i,:) = [p2(1,i), p2(2,i), 1, 0, 0, 0, ...
        -p1(1,i)*p2(1,i), -p1(1,i)*p2(2,i), -p1(1,i)];
end
for i = (size(Q,1) / 2) + 1 : size(Q,1)
    % beta
    j = i - size(p1,2);
    Q(i,:) = [0, 0, 0, p2(1,j), p2(2,j), 1, ...
        -p1(2,j)*p2(1,j), -p1(2,j)*p2(2,j), -p1(2,j)];
end
% decompose Q
[U, S, V] = svd(Q);
% extract h
h = V(:,end);
% restructure h to a 3x3 matrix
H = [h(1), h(2), h(3); h(4), h(5), h(6); h(7), h(8), h(9)];
end