% function compute the dense optical flow mask overlopping to next image
% function newImg = getFloMaskImg(flo, img)
function newImg = getFloMaskImg(flo, img, imgNext)
% load 'imTest.mat'; 
% load 'floTest.mat';
% img = im1;
rows = size(img, 1); cols = size(img, 2); newImg = uint8(zeros(size(img)));
[corr_x,corr_y] = meshgrid(1:cols, 1:rows);
flo_corr_x = corr_x + flo(:,:,1); %flo_corr_x = flo_corr_x(:);
flo_corr_y = corr_y + flo(:,:,2); %flo_corr_y = flo_corr_y(:);
img(1,1) = 0;
lostIdx = unique([find(flo_corr_x<1); find(flo_corr_y<1); ...
    find(flo_corr_x>cols-0.5); find(flo_corr_y>rows-0.5)]);
flo_corr_x(lostIdx) = 1; flo_corr_x = round(flo_corr_x);
flo_corr_y(lostIdx) = 1; flo_corr_y = round(flo_corr_y);

img_r = img(:,:,1); img_g = img(:,:,2); img_b = img(:,:,3);
imgNext_r = imgNext(:,:,1); imgNext_g = imgNext(:,:,2); imgNext_b = imgNext(:,:,3);
newImg_r = uint8(zeros(size(img_r))); 
newImg_g = uint8(zeros(size(img_g))); 
newImg_b = uint8(zeros(size(img_b)));

index_org = sub2ind(size(img_r), corr_y(:), corr_x(:));
index_new = sub2ind(size(newImg_r), flo_corr_y(:), flo_corr_x(:));
newImg_r(index_new) = img_r(index_org); 
newImg_r(find(newImg_r==0)) = imgNext_r(find(newImg_r==0));
newImg_g(index_new) = img_g(index_org); newImg_g(find(newImg_g==0)) = imgNext_g(find(newImg_g==0));
newImg_b(index_new) = img_b(index_org); newImg_b(find(newImg_b==0)) = imgNext_b(find(newImg_b==0));
newImg(:,:,1) = newImg_r; newImg(:,:,2) = newImg_g; newImg(:,:,3) = newImg_b;


figure(22222); subplot(2, 1, 1), imshow(img); subplot(2, 1, 2), imshow(newImg);
figure(22221); subplot(2, 1, 1), imshow(imgNext); subplot(2, 1, 2), imshow(newImg);
end