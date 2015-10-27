%% input image, occluded points and the window size
%% function [msk, mskImg] = buildOcclusionMask(img, pts, wSz)
%%
function [msk, mskImg] = buildOcclusionMask(img, pts, wSz)
msk = zeros(size(img));
if size(pts, 1) ~= length(pts)
    pts = pts';
end

for i= 1:length(pts)
    cx = floor(pts(i,2)); cy = floor(pts(i,1));
    msk(max(cx-wSz, 1):cx+wSz, max(cy-wSz, 1):cy+wSz, :) = 1;
end

mskImg= msk.*img;

end