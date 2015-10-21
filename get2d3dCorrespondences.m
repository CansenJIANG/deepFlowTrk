% Function establish 2d-3d correspondneces based on small projection error
% [pts3D, proj3D, velo_depth] = get2d3dCorrespondences(sequence, frmIdx, prjErrThd)
% 
function [pts3D, prj3D, velo_depth] = get2d3dCorrespondences(sequence, frmIdx, P_velo_to_img,distThd)
base_dir = '/home/jiang/CvDataset/KITTI';
% 2D-3D correspondences within 1.5 pixels
fid  = fopen(sprintf('%s/tracking_module/training/velodyne/%04s/%06d.bin',...
      base_dir, sequence(1:end-1), frmIdx-1),'rb');
velo = fread(fid,[4 inf],'single')';
fclose(fid);

idx = velo(:,1)<0; velo(idx,:) = []; % remove all points behind image plane
idx = velo(:,1)>distThd; velo(idx,:) = []; % remove all points father than 15ms
% project to image plane (exclude luminance)
[velo_img, velo_depth] = project(velo(:,1:3),P_velo_to_img); velo_img = velo_img';

% Lets validate only those 3D and projected points which are within image
% Image size
Ix = 1238; Iy = 374; 
ptList = velo_img(1,:)>0.5 & velo_img(2,:)>0.5 &...
    velo_img(1,:)< (Ix- 0.5) & velo_img(2,:)<(Iy- 0.5);

pts3D   = velo(ptList,1:3)'; velo_depth = velo_depth(ptList);
prj3D  = velo_img(:,ptList);
prj3D(1,prj3D(1, :)<1) = 1; prj3D(2,prj3D(2, :)<1) = 1;
end