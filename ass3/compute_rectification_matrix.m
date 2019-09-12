% H = compute_rectification_matrix(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input: points1, points2 of the form (4,n) 
%        n has to be at least 5
%
% Output:  H (4,4) matrix 
% 

function H = compute_rectification_matrix( points1, points2 )
thismanypoints = size(points1)
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
% define W from points 1 & 2, assuming same size of points 1 & 2
W = zeros(size(p1,2) * 3, 16);
for i = 1 : size(W,1)/3
    W(i*3,:) = [...
        p1(1,i), p1(2,i), p1(3,i), p1(4,i), ...
        0, 0, 0, 0, ...
        0, 0, 0, 0, ...
       -p1(1,i)*p2(1,i), -p1(2,i)*p2(1,i), -p1(3,i)*p2(1,i), -p1(4,i)*p2(1,i)];
    
     W(i*3+1,:) = [...
         0, 0, 0, 0, ...
         p1(1,i), p1(2,i), p1(3,i), p1(4,i), ...
         0, 0, 0, 0, ...
        -p1(1,i)*p2(2,i), -p1(2,i)*p2(2,i), -p1(3,i)*p2(2,i), -p1(4,i)*p2(2,i)];
    
     W(i*3+2,:) = [...
         0, 0, 0, 0, ...
         0, 0, 0, 0, ...
         p1(1,i), p1(2,i), p1(3,i), p1(4,i), ...
        -p1(1,i)*p2(3,i), -p1(2,i)*p2(3,i), -p1(3,i)*p2(3,i), -p1(4,i)*p2(3,i)];
    
end

% decompose W
[U, S, V] = svd(W);
% extract h
h = V(:,end);
% restructure h to a 3x3 matrix
H = [h(1),h(2),h(3),h(4)
     h(5),h(6),h(7),h(8)
     h(9),h(10),h(11),h(12)
     h(13),h(14),h(15),h(16)];
end
        