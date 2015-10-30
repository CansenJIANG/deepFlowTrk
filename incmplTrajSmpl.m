%% Incomplete Trajectories Construction
function paramDF = incmplTrajSmpl(paramDF)
trajlen = paramDF.seqLen; showFig = 0;
nFrm = paramDF.nFrm;
trajAllFwd2d = [];trajAllFwd3d = [];
for i= 1:length(paramDF.ForwardTraj3Dproj)
    trajAllFwd2d = [trajAllFwd2d; paramDF.ForwardTraj3Dproj{i}];
    trajAllFwd3d = [trajAllFwd3d; paramDF.ForwardTraj3D{i}];
end
trajAllNfrmBwd2d = []; trajAllNfrmBwd3d = [];
for i=1:length(paramDF.nFrmBwdTraj3Dproj)
    trajAllNfrmBwd2d = [trajAllNfrmBwd2d; paramDF.nFrmBwdTraj3Dproj{i}];
    trajAllNfrmBwd3d = [trajAllNfrmBwd3d; paramDF.nFrmBwdTraj3D{i}];
end


% Consturcture Incomplete Trajectories Indexing
incmplTrajFwd2d = []; incmplTrajFwd3d = [];
% Get index of features at least observed 5 frames
incmplIdxFwd = setdiff(find(paramDF.ForwardTraj3Dproj{1,nFrm}(1,:)~=1),...
                       find(paramDF.ForwardTraj3Dproj{1,end}(1,:)~=1));

% Get index of features at least observed 5 frames
incmplIdxFwd = intersect( incmplIdxFwd, find(paramDF.nFrmBwdTraj3Dproj{1,paramDF.nExtnd+1}(1,:)~=1));
gndIdxFwd  = intersect(find(paramDF.ForwardTraj3D{1,nFrm}(3,:)>paramDF.gndHight),...
    find(paramDF.nFrmBwdTraj3D{1,paramDF.nExtnd+1}(3,:)>paramDF.gndHight));
incmplIdxFwd = intersect(incmplIdxFwd, gndIdxFwd);

trajAllFwd2d = trajAllFwd2d(:,incmplIdxFwd);
trajAllFwd3d = trajAllFwd3d(:,incmplIdxFwd); 
trajAllNfrmBwd2d = trajAllNfrmBwd2d(:,incmplIdxFwd); 
trajAllNfrmBwd3d = trajAllNfrmBwd3d(:,incmplIdxFwd); 

% remove lost tracking index
outIdxFwdNfrmBwd = [];
for i=1:size(trajAllFwd2d,2)
    tmpTrajAllFwd2d = trajAllFwd2d(1:2*nFrm,i);
    if(~isempty(find(tmpTrajAllFwd2d==1)))
        outIdxFwdNfrmBwd = [outIdxFwdNfrmBwd, i];
        continue;
    end
    tmpTrajAllNfrmBwd2d = trajAllNfrmBwd2d(1:2*(paramDF.nExtnd+1),i);
    if(~isempty(find(tmpTrajAllNfrmBwd2d==1)))
        outIdxFwdNfrmBwd = [outIdxFwdNfrmBwd, i];
    end
end
trajAllFwd2d(:,outIdxFwdNfrmBwd) = [];
trajAllFwd3d(:,outIdxFwdNfrmBwd) = []; 
trajAllNfrmBwd2d(:,outIdxFwdNfrmBwd) = []; 
trajAllNfrmBwd3d(:,outIdxFwdNfrmBwd) = []; 

% save inlier incomplete trajectories
idxFilled = [];
for i = paramDF.nExtnd:paramDF.seqLen
    idxFill = find(trajAllFwd2d(2*i,:)==1);
    idxFill = setdiff(idxFill, idxFilled);
    incmplTrajFwd2dTmp = trajAllFwd2d(1:2*(i-1),idxFill);
    incmplTrajNfrmFwd2d = trajAllNfrmBwd2d(3:2*(trajlen-i+2),idxFill);
    incmplTrajNfrmFwd2d = reorderMatSeq(incmplTrajNfrmFwd2d, 2);
    incmplTrajFwd2dTmp = [incmplTrajNfrmFwd2d; incmplTrajFwd2dTmp];
    incmplTrajFwd2d = [incmplTrajFwd2d, incmplTrajFwd2dTmp];
    incmplTrajFwd3dTmp = trajAllFwd3d(1:3*(i-1),idxFill);
    incmplTrajNfrmFwd3d = trajAllNfrmBwd3d(4:3*(trajlen-i+2),idxFill);
    incmplTrajNfrmFwd3d = reorderMatSeq(incmplTrajNfrmFwd3d, 3);
    incmplTrajFwd3dTmp = [incmplTrajNfrmFwd3d; incmplTrajFwd3dTmp];
    incmplTrajFwd3d = [incmplTrajFwd3d, incmplTrajFwd3dTmp];
    idxFilled = unique([idxFill,idxFilled]);
end
%% =======================================================================
trajAllBwd2d = [];trajAllBwd3d = [];
incmplTrajBwd2d = []; incmplTrajBwd3d = []; 
for i= 1:length(paramDF.ForwardTraj3Dproj)
    trajAllBwd2d = [trajAllBwd2d; paramDF.BackwardTraj3Dproj{i}];
    trajAllBwd3d = [trajAllBwd3d; paramDF.BackwardTraj3D{i}];
end
trajAllNfrmFwd2d = []; trajAllNfrmFwd3d = [];
for i=1:length(paramDF.nFrmFwdTraj3Dproj)
    trajAllNfrmFwd2d = [trajAllNfrmFwd2d; paramDF.nFrmFwdTraj3Dproj{i}];
    trajAllNfrmFwd3d = [trajAllNfrmFwd3d; paramDF.nFrmFwdTraj3D{i}];
end

% Consturcture Incomplete Trajectories Indexing
incmplIdxBwd = setdiff(find(paramDF.BackwardTraj3Dproj{1,nFrm}(1,:)~=1),...
                       find(paramDF.BackwardTraj3Dproj{1,end}(1,:)~=1));
                     
incmplIdxBwd = intersect( incmplIdxBwd, find(paramDF.nFrmFwdTraj3Dproj{1,paramDF.nExtnd+1}(1,:)~=1));
gndIdxBwd  = intersect(find(paramDF.BackwardTraj3D{1,nFrm}(3,:)>paramDF.gndHight),...
    find(paramDF.nFrmFwdTraj3D{1,paramDF.nExtnd+1}(3,:)>paramDF.gndHight));
incmplIdxBwd = intersect(incmplIdxBwd, gndIdxBwd);

trajAllBwd2d = trajAllBwd2d(:,incmplIdxBwd);
trajAllBwd3d = trajAllBwd3d(:,incmplIdxBwd); 
trajAllNfrmFwd2d = trajAllNfrmFwd2d(:,incmplIdxBwd); 
trajAllNfrmFwd3d = trajAllNfrmFwd3d(:,incmplIdxBwd); 

% remove lost tracking index
outIdxBwdNfrmFwd = [];
for i=1:size(trajAllBwd2d,2)
    tmpTrajAllBwd2d = trajAllBwd2d(1:2*nFrm,i);
    if(~isempty(find(tmpTrajAllBwd2d==1)))
        outIdxBwdNfrmFwd = [outIdxBwdNfrmFwd, i];
        continue;
    end
    tmpTrajAllNfrmFwd2d = trajAllNfrmFwd2d(1:2*(paramDF.nExtnd+1),i);
    if(~isempty(find(tmpTrajAllNfrmFwd2d==1)))
        outIdxBwdNfrmFwd = [outIdxBwdNfrmFwd, i];
    end
end
trajAllBwd2d(:,outIdxBwdNfrmFwd) = [];
trajAllBwd3d(:,outIdxBwdNfrmFwd) = []; 
trajAllNfrmFwd2d(:,outIdxBwdNfrmFwd) = []; 
trajAllNfrmFwd3d(:,outIdxBwdNfrmFwd) = []; 

idxFilled = [];
for i = paramDF.nExtnd:paramDF.seqLen
    idxFill = find(trajAllBwd2d(2*i,:)==1);
    idxFill = setdiff(idxFill, idxFilled);
    incmplTrajBwd2dTmp = trajAllBwd2d(1:2*(i-1),idxFill);
    incmplTrajNfrmBwd2d = trajAllNfrmFwd2d(3:2*(trajlen-i+2),idxFill);
    incmplTrajBwd2dTmp = reorderMatSeq(incmplTrajBwd2dTmp, 2);
    incmplTrajBwd2dTmp = [incmplTrajBwd2dTmp; incmplTrajNfrmBwd2d];
    incmplTrajBwd2d = [incmplTrajBwd2d, incmplTrajBwd2dTmp];
    incmplTrajBwd3dTmp = trajAllBwd3d(1:3*(i-1),idxFill);
    incmplTrajNfrmBwd3d = trajAllNfrmFwd3d(4:3*(trajlen-i+2),idxFill);
    incmplTrajBwd3dTmp = reorderMatSeq(incmplTrajBwd3dTmp, 3);
    incmplTrajBwd3dTmp = [incmplTrajBwd3dTmp; incmplTrajNfrmBwd3d];
    incmplTrajBwd3d = [incmplTrajBwd3d, incmplTrajBwd3dTmp];
    idxFilled = unique([idxFill,idxFilled]);
end


if showFig
    incmplImgFwdSta = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.staSeq-paramDF.nExtnd+2).name];
    incmplImgFwdEnd = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.staSeq+nFrm).name];
    showFeatureMatchesVertical(imread(incmplImgFwdSta), imread(incmplImgFwdEnd),...
        incmplTrajFwd2d(1:2,:)', incmplTrajFwd2d(end-1:end,:)');
     
    incmplImgBwdSta = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.endSeq-nFrm).name];
    incmplImgBwdEnd = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.endSeq+paramDF.nExtnd-1).name];
    showFeatureMatchesVertical(imread(incmplImgBwdSta), imread(incmplImgBwdEnd),...
        incmplTrajBwd2d(1:2,:)', incmplTrajBwd2d(end-1:end,:)');
end
%%
%%
%%
%% Trajectories Sampling
rejectIdx = [];
for i=1:size(incmplTrajFwd2d,2)
    trajTmp = incmplTrajFwd2d(:,i);
    if(sum(trajTmp==1))
        rejectIdx = [rejectIdx, i];
    end
end
incmplTrajFwd2d(:,rejectIdx) = []; incmplTrajFwd3d(:,rejectIdx) = [];
lenExt = trajlen -5;
floNameSta = ['./flo/', paramDF.sequence(1:end-1),sprintf('_%06d_%06d.flo',paramDF.staSeq-trajlen + 7, paramDF.staSeq-trajlen + 8)];
floSta = readFlowFile(floNameSta); paramDF.incmplTrajNbSta = 60;
[inlierIdx, sampleIdx, ourlierIdx] = flowSampling(incmplTrajFwd3d, incmplTrajFwd2d, floSta, paramDF.incmplTrajNbSta);
paramDF.incmplTrajFwd2d = ones([size(paramDF.cmplTrajFwd2d,1), length(sampleIdx)]);
paramDF.incmplTrajFwd3d = ones([size(paramDF.cmplTrajFwd3d,1), length(sampleIdx)]);
paramDF.incmplTrajFwd2d(1:size(incmplTrajFwd2d,1), :) = incmplTrajFwd2d(:, sampleIdx);
paramDF.incmplTrajFwd3d(1:size(incmplTrajFwd3d,1), :) = incmplTrajFwd3d(:, sampleIdx);
if showFig
    incmplImgFwdSta = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.staSeq-trajlen + 7).name];
    incmplImgFwdEnd = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.staSeq+6).name];
    showFeatureMatchesVertical(imread(incmplImgFwdSta), imread(incmplImgFwdEnd),...
        incmplTrajFwd2d(1:2,:)', incmplTrajFwd2d(end-1:end,:)');title('Incomplete Forward No Sampling');
    showFeatureMatchesVertical(imread(incmplImgFwdSta), imread(incmplImgFwdEnd),...
        incmplTrajFwd2d(1:2,inlierIdx)', incmplTrajFwd2d(end-1:end,inlierIdx)');title('Incomplete Forward InlierIdx');
    showFeatureMatchesVertical(imread(incmplImgFwdSta), imread(incmplImgFwdEnd),...
        incmplTrajFwd2d(1:2,sampleIdx)', incmplTrajFwd2d(end-1:end,sampleIdx)');title('Incomplete Forward SmpIdx');
    showFeatureMatchesVertical(imread(incmplImgFwdSta), imread(incmplImgFwdEnd),...
        incmplTrajFwd2d(1:2,ourlierIdx)', incmplTrajFwd2d(end-1:end,ourlierIdx)');title('Incomplete Forward OutlierIdx');
end
paramDF.incmplTrajFwd2d = incmplTrajFwd2d(:, sampleIdx);
paramDF.incmplTrajFwd3d = incmplTrajFwd3d(:, sampleIdx);

floNameEnd = ['./flo/',paramDF.sequence(1:end-1),sprintf('_%06d_%06d.flo', paramDF.endSeq+lenExt-1, paramDF.endSeq+lenExt)];
floEnd = readFlowFile(floNameEnd); paramDF.incmplTrajNbEnd = 60;
[inlierIdx, sampleIdx, outlierIdx] = flowSampling(incmplTrajBwd3d, incmplTrajBwd2d, floEnd, paramDF.incmplTrajNbEnd);
if showFig
    incmplImgBwdSta = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.endSeq-trajlen + 7).name];
    incmplImgBwdEnd = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.endSeq+lenExt).name];
    showFeatureMatchesVertical(imread(incmplImgBwdSta), imread(incmplImgBwdEnd),...
        incmplTrajBwd2d(1:2,:)', incmplTrajBwd2d(end-1:end,:)'); title('Incomplete Backward No Sampling');
    showFeatureMatchesVertical(imread(incmplImgBwdSta), imread(incmplImgBwdEnd),...
        incmplTrajBwd2d(1:2,inlierIdx)', incmplTrajBwd2d(end-1:end,inlierIdx)'); title('Incomplete Backward InlierIdx');
    showFeatureMatchesVertical(imread(incmplImgBwdSta), imread(incmplImgBwdEnd),...
        incmplTrajBwd2d(1:2,sampleIdx)', incmplTrajBwd2d(end-1:end,sampleIdx)'); title('Incomplete Backward SmpIdx');
    showFeatureMatchesVertical(imread(incmplImgBwdSta), imread(incmplImgBwdEnd),...
        incmplTrajBwd2d(1:2,ourlierIdx)', incmplTrajBwd2d(end-1:end,ourlierIdx)'); title('Incomplete Backward OutlierIdx');
end
paramDF.incmplTrajBwd2d = incmplTrajBwd2d(:, sampleIdx);
paramDF.incmplTrajBwd3d = incmplTrajBwd3d(:, sampleIdx);
incmplImgBwdSta = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.endSeq-5).name];
figure,imshow(imread(incmplImgBwdSta));hold on; plot2dTrajectories(paramDF.incmplTrajFwd2d, 1); 

end