function  [newPts2d, newPts3d, lostIdx] = decreaseNeighborhood(pts2d, pts3d, wSz)
mask_row = 2*[-1, -1, -1;0, 0, 0; 1,1,1]; 
mask_col = [-1, -1, -1;0, 0, 0; 1,1,1]';
flag2d = 0; flag3d = 0;
if(size(pts2d,1)~=length(pts2d)); pts2d = pts2d'; flag2d = 1; end
if(size(pts3d,1)~=length(pts3d)); pts3d = pts3d'; flag3d = 1; end

newPts2d = zeros(length(pts2d(:,1))/9, 2); 
newPts3d = zeros(length(pts3d(:,1))/9, 3);
pts2d_row = reshape(pts2d(:,1), [9, length(pts2d(:,1))/9]); 
pts2d_col = reshape(pts2d(:,2), [9, length(pts2d(:,1))/9]);

lostIdx = find(sum(pts2d_row)==9); newPts2d(lostIdx, :) = 1;
inlierIdx = find(pts2d_row(5,:)~=1); 
newPts2d(inlierIdx, 1) = pts2d_row(5,inlierIdx)';
newPts2d(inlierIdx, 2) = pts2d_col(5,inlierIdx)';
mixIdx = 1:length(pts2d(:,1))/9; mixIdx([inlierIdx,lostIdx]) = [];
mixPts2d = pts2d_row(:, mixIdx); 
for i = 1:4
    iup = 5-i; idown = 5+i;
    interIdx = mixIdx(find(mixPts2d(iup, :)~=1));
    newPts2d(interIdx, 1) = pts2d_row(iup,interIdx)';
    newPts2d(interIdx, 2) = pts2d_col(iup,interIdx)';
    mixIdx(find(mixPts2d(iup, :)~=1)) = [];
    mixPts2d(:, find(mixPts2d(iup, :)~=1)) = [];
    interIdx = mixIdx(find(mixPts2d(idown, :)~=1));
    newPts2d(interIdx, 1) = pts2d_row(idown,interIdx)';
    newPts2d(interIdx, 2) = pts2d_col(idown,interIdx)';
    mixIdx(find(mixPts2d(idown, :)~=1)) = [];
    mixPts2d(:, find(mixPts2d(idown, :)~=1)) = [];
    if length(mixIdx)==0
        break;
    end
end
% reshape 3d
pts3d_x = reshape(pts3d(:,1), [9, length(pts3d(:,1))/9]); 
pts3d_y = reshape(pts3d(:,2), [9, length(pts3d(:,2))/9]);  
pts3d_z = reshape(pts3d(:,3), [9, length(pts3d(:,3))/9]); 
newPts3d = [pts3d_x(1, :)', pts3d_y(1, :)', pts3d_z(1, :)'];
newPts3d(lostIdx, :) = 0;
if flag2d; newPts2d = newPts2d'; end
if flag3d; newPts3d = newPts3d'; end
end


