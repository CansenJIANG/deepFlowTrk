function [fuseMskImg,  motMskImg, retrkIdx, lostTrkIdx] = motionInterpolation(paramDF, trkOF, lostIdx, lostMskImg, currImg,mskPos, nextImg)
motMskImg = zeros(size(currImg)); wSz = paramDF.wSz; retrkIdx = [];
if size(mskPos,1)~= length(mskPos); mskPos = mskPos';end
% figure, imshow(currImg), hold on;
for i=1:length(lostIdx)
    rnnIdx2d = paramDF.rnnIdxTab2d{lostIdx(i)};   rnnIdx2d(1)  = [];
    rnnIdx3d = paramDF.rnnIdxTab3d{lostIdx(i)};   rnnIdx3d(1)  = [];
    rnnDist2d = paramDF.rnnDistTab2d{lostIdx(i)}; rnnDist2d(1) = [];
    rnnDist3d = paramDF.rnnDistTab3d{lostIdx(i)}; rnnDist3d(1) = [];
    
    [rnnIdx2d, filterIdx] = intersect(rnnIdx2d,rnnIdx3d); rnnDist2d = rnnDist2d(filterIdx);
    [rnnIdx2d, filterIdx] = setdiff(rnnIdx2d,lostIdx); rnnDist2d = rnnDist2d(filterIdx);
    if length(rnnIdx2d)<1
        continue;
    end
    sumDist2d = sum(rnnDist2d);
    if size(rnnDist2d, 2) == 1
        scl2d = 1;
    else
        scl2d = sumDist2d*ones(size(rnnDist2d));
        scl2d = (scl2d - rnnDist2d)/sumDist2d; %scl2d(1) = 0;
    end
    interp2dFlo_x = scl2d*trkOF(rnnIdx2d, 1);
    interp2dFlo_y = scl2d*trkOF(rnnIdx2d, 2);
   
%     
%     plot(paramDF.keyFrmSta3dPrj(1, lostIdx(i)),  paramDF.keyFrmSta3dPrj(2, lostIdx(i)), 'sr');
%     plot(paramDF.keyFrmSta3dPrj(1, lostIdx(i))+interp2dFlo_x,...
%          paramDF.keyFrmSta3dPrj(2, lostIdx(i))+interp2dFlo_y, 'sg');
    
    cx = floor(mskPos(i,2)); cy = floor(mskPos(i,1));
    flo_cx = floor( mskPos(i,2)+interp2dFlo_x ); 
    flo_cy = floor( mskPos(i,1)+interp2dFlo_y );
    if (flo_cy<wSz | flo_cx<wSz | flo_cx > size(motMskImg,1)-wSz | flo_cy > size(motMskImg,2)-wSz |...
        cy<wSz | cx<wSz | cx > size(motMskImg,1)-wSz | cy > size(motMskImg,2)-wSz)
        
        continue;
    end
    if i == 3512;
        display(i);
    end
    motMskImg(max(flo_cx-wSz, 1):max(flo_cx+wSz, 1), max(flo_cy-wSz, 1):max(flo_cy+wSz,1), :) =...
    lostMskImg(max(cx-wSz, 1):cx+wSz, max(cy-wSz, 1):cy+wSz, :);
    retrkIdx = [retrkIdx; i];
end
% figure, subplot(2,1,1), imshow(lostMskImg); hold on; subplot(2,1,2), imshow(motMskImg); 
fuseMskImg = nextImg;
fuseMskImg(motMskImg~=0) = motMskImg(motMskImg~=0);
figure, imshow(fuseMskImg);
retrkIdx = lostIdx(retrkIdx)';
lostTrkIdx = setdiff(lostIdx, retrkIdx)';
end