function  [newPts2d, newPts3d] = increaseNeighborhood(pts2d, pts3d, wSz)
mask_row = 2*[-1, -1, -1;0, 0, 0; 1,1,1]; 
mask_col = [-1, -1, -1;0, 0, 0; 1,1,1]';
flag2d = 0;
flag3d = 0;
newPts2d = []; newPts3d = [];
if(size(pts2d,1)~=length(pts2d)); pts2d = pts2d'; flag2d = 1; end
if(size(pts3d,1)~=length(pts3d)); pts3d = pts3d'; flag3d = 1; end
% for i = 1:length(pts2d)
%         newRow = mask_row + round(pts2d(i,1));
%         newCol = mask_col + round(pts2d(i,2));
%         newPts2d = [newPts2d; [newRow(:), newCol(:)]];
%         newPts3d = [newPts3d; repmat(pts3d(i, :), [9, 1])];
% end

pts2d_row = repmat(pts2d(:,1)', [length(mask_row(:)), 1]); 
mask_row  = repmat(mask_row(:), [1, length(pts2d(:,1)) ]);
pts2d_col = repmat(pts2d(:,2)', [length(mask_col(:)), 1]);
mask_col  = repmat(mask_col(:), [1, length(pts2d(:,2)) ]);
newPts2d_row = pts2d_row - mask_row;
newPts2d_col = pts2d_col - mask_col;
newPts2d = [newPts2d_row(:), newPts2d_col(:)];

mask_row = [-1, -1, -1; 0, 0, 0; 1,1,1]; 
pts3d_x = repmat(pts3d(:,1)', [length(mask_row(:)), 1]); 
pts3d_y = repmat(pts3d(:,2)', [length(mask_row(:)), 1]); 
pts3d_z = repmat(pts3d(:,3)', [length(mask_row(:)), 1]); 
newPts3d = [pts3d_x(:), pts3d_y(:), pts3d_z(:)];

if flag2d; newPts2d = newPts2d'; end
if flag3d; newPts3d = newPts3d'; end
end