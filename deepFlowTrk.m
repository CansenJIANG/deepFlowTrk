%% Deep Flow Tracking in Multiple Frames
%%
%% 1. Deep Matching for two key frames
%% 2. Two side optical flow to build complete and incomplete trajectories
%% 3. Select good trajectories with likelihood

%%
%%
%%
%% Add library root
clear; clc; close all;
addpath(genpath('/home/jiang/CvTools/DenseOpticalFlow/Piecewise_Rigid_Scene_Model/DataFlow'));
addpath(genpath('/home/jiang/CvDataset/KITTI/devkit/devkit_raw/matlab'));
addpath(genpath('/home/jiang/CvDataset/KITTI/devkit/devkit_tracking/matlab'));
addpath(genpath('/home/jiang/CvTools/MotionSegExp'));
addpath(genpath('/home/jiang/CvTools/deepmatching_1.2.2_c++'))
addpath(genpath('/home/jiang/CvTools/flow-code-matlab'))
%%
%%
%%
%% configure Deep Flow Algorithm
deepFlowTrkSetting;
leftImgDir = paramDF.leftImgDir;
% save Result name
paramDF.traj3DName = ['traj3D_',leftImgDir(end-8:end-1),'_',paramDF.sequence(1:end-1),'_',...
    num2str(paramDF.staSeq),'_',num2str(endSeq),'.mat'];
paramDF.traj3DprojName = ['traj3Dproj_',leftImgDir(end-8:end-1),'_',paramDF.sequence(1:end-1),'_',...
    num2str(paramDF.staSeq),'_',num2str(endSeq),'.mat'];
paramDF.lostTraj3DName = ['lostTraj3D_image_02_', paramDF.sequence(1:4),'_',num2str(paramDF.staSeq),...
    '_',num2str(paramDF.endSeq),'.mat'];

%%
%%
%%
%% load calibration
paramCalib = [];
calibFile = [ paramDF.calib_dir,  paramDF.sequence(1:4),'.txt'];
fid = fopen(fullfile(calibFile),'r');
P2  = readVariable(fid,'P2',3,4);
P3  = readVariable(fid,'P3',3,4);
Tr  = readVariable(fid,'Tr_velo_cam',3,4);
R_rect =  readVariable(fid,'R_rect',3,3);
fclose(fid);

% Get intrinsic parameters
K2 = P2(1:3, 1:3);
K3 = P3(1:3, 1:3);

% To project a point from Velodyne coordinates into the left color image,
% you can use this formula: x = P2 * R0_rect * Tr_velo_to_cam * y
% For the right color image: x = P3 * R0_rect * Tr_velo_to_cam * y
R_rect(4,4) = 1;
P_velo_to_img = P2*R_rect*[Tr; 0, 0, 0, 1];
paramDF.P_velo_to_img = P_velo_to_img;
%%
%%
%%
%% Deep Matching for keyframe matching
if 1
    % run deep matching algorithm
    showFig = 0;
    imgSta = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.staSeq).name];
    imgEnd = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.endSeq).name];
    imgMid = [paramDF.leftFileDir, paramDF.imgsLeft(paramDF.midSeq).name];
    command  = [paramDF.deepMatchingExe,imgSta,' ', imgEnd, ' -nt 0 -out ', paramDF.matchName];
    tic; [status,cmdout] = system(command); toc;
    paramDF.matches = load(paramDF.matchName);
    scoreThd = paramDF.matches(:,6); scoreThd = sort(scoreThd);
    scoreThd = scoreThd(floor(1*length(scoreThd)));
    paramDF.matches =  paramDF.matches( paramDF.matches(:,6)<scoreThd,:);
    
    % get 2d-3d correspondences of key frames
    [paramDF.keyFrmSta3d, paramDF.keyFrmSta3dPrj] = get2d3dCorrespondences(...
        paramDF.sequence, paramDF.staSeq, paramDF.P_velo_to_img, paramDF.distThd);
    % figure, imshow(imread(imgSta)), hold on; plot(paramDF.keyFrmSta3dPrj(1,:),paramDF.keyFrmSta3dPrj(2,:),'.r')
    [paramDF.keyFrmEnd3d, paramDF.keyFrmEnd3dPrj] = get2d3dCorrespondences(...
        paramDF.sequence, paramDF.endSeq, paramDF.P_velo_to_img, paramDF.distThd);
    
    % get matches within 2d-3d correspondences
    [paramDF.DM2d3dSta(:,1), paramDF.DM2d3dSta(:,2)] = ...
        knnsearch(paramDF.keyFrmSta3dPrj', paramDF.matches(:, 1:2));
    outlierIdx = paramDF.DM2d3dSta(:,2)>paramDF.prjErrThd;
    paramDF.DM2d3dSta(outlierIdx,:) = [];
    paramDF.matches(outlierIdx,:) = [];
    
    [paramDF.DM2d3dEnd(:,1), paramDF.DM2d3dEnd(:,2)] = ...
        knnsearch(paramDF.keyFrmEnd3dPrj', paramDF.matches(:, 3:4));
    outlierIdx = paramDF.DM2d3dEnd(:,2)>paramDF.prjErrThd;
    paramDF.DM2d3dEnd(outlierIdx,:) = [];
    paramDF.matches(outlierIdx,:) = [];
    paramDF.DM2d3dSta(outlierIdx,:) = [];
    if showFig
        figure,imshow(imread(imgSta)), hold on;
        plot(paramDF.matches(:,1), paramDF.matches(:,2), 'sr');
        plot(paramDF.keyFrmSta3dPrj(1, paramDF.DM2d3dSta(:, 1))',...
            paramDF.keyFrmSta3dPrj(2, paramDF.DM2d3dSta(:, 1))', '.g');
        figure,imshow(imread(imgEnd)), hold on;
        plot(paramDF.matches(:,3), paramDF.matches(:,4), 'sr');
        plot(paramDF.keyFrmEnd3dPrj(1, paramDF.DM2d3dEnd(:, 1))',...
            paramDF.keyFrmEnd3dPrj(2, paramDF.DM2d3dEnd(:, 1))', '.g');
        
        showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
            paramDF.keyFrmSta3dPrj(:, paramDF.DM2d3dSta(:, 1))', ...
            paramDF.keyFrmEnd3dPrj(:, paramDF.DM2d3dEnd(:, 1))');
        showFeatureVertical(imread(imgSta), imread(imgEnd),...
            paramDF.keyFrmSta3dPrj(:, paramDF.DM2d3dSta(:, 1))', ...
            paramDF.keyFrmEnd3dPrj(:, paramDF.DM2d3dEnd(:, 1))');
        
        showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
            paramDF.matches(paramDF.matches(:,6)<scoreThd, 1:2), ...
            paramDF.matches(paramDF.matches(:,6)<scoreThd, 3:4));
    end
    %%
    %%
    %%
    %% Start OF tracking
    tic; paramDF = SequentialOpticalFlowTrking(paramDF, 1); toc;
    tic; paramDF = SequentialOpticalFlowTrking(paramDF, 2); toc;
    tic; paramDF = SequentialOpticalFlowTrking(paramDF, 3); toc;
    tic; paramDF = SequentialOpticalFlowTrking(paramDF, 4); toc;
    save('flowDone.mat')
else
    load('flowDone.mat');
end

%%
%%
%%
%% Complete Trajectories Validation
% fwdLastFlo = paramDF.ForwardTraj3Dproj{1,end}(:, paramDF.DM2d3dSta(:, 1));
% showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
%     fwdLastFlo', ...
%     paramDF.ForwardTraj3Dproj{1,1}(:, paramDF.DM2d3dSta(:, 1))');

% bwdLastFlo = paramDF.BackwardTraj3Dproj{1,end}(:, paramDF.DM2d3dEnd(:, 1));
% showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
%     bwdLastFlo', ...
%     paramDF.BackwardTraj3Dproj{1,1}(:, paramDF.DM2d3dEnd(:, 1))');

% fwdLastFloDist = distances2vectors(fwdLastFlo, paramDF.matches(:, 3:4));
% bwdLastFloDist = distances2vectors(bwdLastFlo, paramDF.matches(:, 1:2));
% paramDF.prjErrThd = 3000;
% if showFig
%     showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
%         paramDF.matches(fwdLastFloDist<paramDF.prjErrThd,1:2),...
%         paramDF.matches(fwdLastFloDist<paramDF.prjErrThd,3:4)');
%     showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
%         paramDF.matches(bwdLastFloDist<paramDF.prjErrThd,1:2),...
%         paramDF.matches(bwdLastFloDist<paramDF.prjErrThd,3:4)');
% end

%% Complete Trajectories Sampling
cmplTrajFwd3d = []; cmplTrajFwd2d = []; gndIdxFwd = [];
unTrkIdx = [];
for i = 1:length(paramDF.ForwardTraj3D)
    cmplTrajFwd3d = [cmplTrajFwd3d; paramDF.ForwardTraj3D{1,i}];
    cmplTrajFwd2d = [cmplTrajFwd2d; paramDF.ForwardTraj3Dproj{1,i}];
    gndIdxFwd = unique([gndIdxFwd, find(paramDF.ForwardTraj3D{1,i}(3, :)<paramDF.gndHight)]);
    unTrkIdx = unique([unTrkIdx, find(paramDF.ForwardTraj3Dproj{1,i}(1, :)==1)]);
end
paramDF.ForwardLostIdx = find(cmplTrajFwd2d(1,:)==1)';
unTrkIdx = unique([gndIdxFwd, unTrkIdx]);
cmplTrajFwd3d(:, unique([paramDF.ForwardLostIdx; unTrkIdx'])) = [];
cmplTrajFwd2d(:, unique([paramDF.ForwardLostIdx; unTrkIdx'])) = [];

floNameSta = ['./flo/',paramDF.sequence(1:end-1),sprintf('_%06d_%06d.flo',paramDF.staSeq-1, paramDF.staSeq)];
floSta = readFlowFile(floNameSta); paramDF.cmplTrajNb = 100;
[inlierIdx, sampleIdxCmpl, outlierIdx] = flowSampling(cmplTrajFwd3d, cmplTrajFwd2d, floSta, paramDF.cmplTrajNb);
if showFig
    showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
        cmplTrajFwd2d(1:2,inlierIdx)', cmplTrajFwd2d(end-1:end,inlierIdx)'); title('inlierIdx');
    showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
        cmplTrajFwd2d(1:2,outlierIdx)', cmplTrajFwd2d(end-1:end,outlierIdx)'); title('outlierIdx');
    showFeatureMatchesVertical(imread(imgSta), imread(imgEnd),...
        cmplTrajFwd2d(1:2,sampleIdxCmpl)', cmplTrajFwd2d(end-1:end,sampleIdxCmpl)');title('sample Idx');
end
paramDF.cmplTrajFwd3d = cmplTrajFwd3d(:,sampleIdxCmpl); paramDF.cmplTrajFwd2d = cmplTrajFwd2d(:,sampleIdxCmpl);

%%
%%
%%
paramDF = incmplTrajSmpl(paramDF);

%% View Trajectories
figure, imshow(imgSta), hold on;
plot(paramDF.cmplTrajFwd2d(1,:),   paramDF.cmplTrajFwd2d(2,:), 'sr');
plot(paramDF.incmplTrajFwd2d(1,:), paramDF.incmplTrajFwd2d(2,:), 'sg');
plot(paramDF.incmplTrajBwd2d(end-1,:), paramDF.incmplTrajBwd2d(end,:), 'sb');

paramDF.traj3dAll = [paramDF.cmplTrajFwd3d, paramDF.incmplTrajFwd3d, paramDF.incmplTrajBwd3d];
paramDF.traj2dAll = [paramDF.cmplTrajFwd2d, paramDF.incmplTrajFwd2d, paramDF.incmplTrajBwd2d];
% 
% paramDF.traj3dAll = paramDF.incmplTrajBwd3d;
% paramDF.traj2dAll = paramDF.incmplTrajBwd2d;

% incmplTraj3dFwdPcd = []; incmplTraj3dBwdPcd = [];
% for i = 1:paramDF.incmplTrajLnth
%     incmplTraj3dFwdPcd = [incmplTraj3dFwdPcd, paramDF.incmplTrajFwd3d(i*3-2:i*3, :)];
%     incmplTraj3dBwdPcd = [incmplTraj3dBwdPcd, paramDF.incmplTrajBwd3d(i*3-2:i*3, :)];
% end
% savepcd('incmplTraj3dFwdPcd.pcd', incmplTraj3dFwdPcd);
% savepcd('incmplTraj3dBwdPcd.pcd', incmplTraj3dBwdPcd);
% cmplTraj3dPcd = [];
% for i = 1:length(paramDF.ForwardTraj3D)
%     cmplTraj3dPcd = [cmplTraj3dPcd,paramDF.cmplTrajFwd3d(i*3-2:i*3, :)];
% end
% savepcd('cmplTraj3dPcd.pcd', cmplTraj3dPcd);
%% normalize data
msRange = 1:size(paramDF.traj3dAll,2);
traj3d = paramDF.traj3dAll; %(:,msRange);
meanTraj3d = mean(traj3d,2);
meanTraj3d = mean(reshape(meanTraj3d,[3, length(meanTraj3d)/3]),2);
traj3d = traj3d - repmat(meanTraj3d, [size(traj3d,1)/3, size(traj3d,2)]);
normNFrame = []; dim = 3;traj3dAllSeq=[];
for i = 1:size(traj3d,1)/dim
    traj1Frame = traj3d(dim*i-dim+1:dim*i, :);
    normNFrame = [normNFrame; sqrt(sum(traj1Frame.^2,1))];
    traj3dAllSeq = [traj3dAllSeq, paramDF.traj3dAll(dim*i-dim+1:dim*i, :)];
end
normNFrame = sum(normNFrame(:))/size(normNFrame,1)/size(normNFrame,2);
traj3d = traj3d*sqrt(3)/normNFrame;
traj3DValidName = ['MS3dPc_02_', paramDF.sequence(1:4),'_',num2str(paramDF.staSeq),...
    '_',num2str(paramDF.endSeq),'.pcd'];
savepcd(traj3DValidName, traj3dAllSeq);

%%
%%
%%
%% Motion Segmentation
%% Sparce Spectural Clustering
MSresultName3D = ['msResult_', paramDF.sequence(1:4),'_',num2str(paramDF.staSeq),...
    '_',num2str(paramDF.endSeq),'.pcd'];
resultName = dir(MSresultName3D);
subspaceDim = 6;
nClass = 3;
if ~isempty(resultName)
    load(resultName.name);
    load(traj3DValidName);
    load(traj3DValidprojName);
    
else
    tic;
    %resultName = [dataName(1:end-4),'_L1ED_',num2str(n),'grps.mat'];
    Cst3d = 0; %Enter 1 to use the additional affine constraint sum(c) == 1
    OptM3d = 'L1ED'; %OptM can be {'L1Perfect','L1Noisy','Lasso','L1ED'}
    lambda = 0.001; %Regularization parameter in 'Lasso' or the noise level for 'L1Noise'
    
    CMat3d = SparseCoefRecovery(traj3d,Cst3d,OptM3d,lambda);
    
    CKSym3d = BuildAdjacency(CMat3d,subspaceDim);
    
    [Grps3d , SingVals3d, LapKernel3d] = SpectralClustering(CKSym3d,nClass);
    toc;
    
    save(MSresultName3D, 'Grps3d');
end

%% regrouping based on 3d distance
if 0
    for nGrp3d = 1:3
        x_all = []; y_all = []; z_all = [];
        for i = 1:size(traj3d,1)/3
            x = traj3d(3*i-2, :)*normNFrame/sqrt(3) + ...
                repmat(meanTraj3d(1),[1, size(traj3d,2)]);
            y = traj3d(3*i-1, :)*normNFrame/sqrt(3) + ...
                repmat(meanTraj3d(2),[1, size(traj3d,2)]);
            z = traj3d(3*i, :)*normNFrame/sqrt(3) + ...
                repmat(meanTraj3d(3),[1, size(traj3d,2)]);
            x_all = [x_all; x]; y_all = [y_all; y]; z_all = [z_all; z];
        end
        x_med = median(x_all); y_med = median(y_all); z_med = median(z_all);
        Grps3 = Grps3d(:,nGrp3d);
        pc_med = [x; y; z];
        
        % take biggest standard deviation class as background
        for kGrp = 1:nClass
            stdvGrp(kGrp, :) = std(pc_med(:, Grps3==kGrp)');
        end
        stdNorm = sqrt(sum((stdvGrp.*stdvGrp)'));
        bkgIdx = find(stdNorm == max(stdNorm));
        
        if 0
            % remove outliers using center to point distance
            for kGrp = 1:nClass
                if kGrp == bkgIdx
                    continue;
                end
                clusterTmp = pc_med(:, Grps3==kGrp);
                cenTmp = mean(clusterTmp');
                cenDist = clusterTmp - repmat(cenTmp', [1, size(clusterTmp,2)]);
                cenDist = sqrt(sum(cenDist.*cenDist))';
                newClassIdx = kGrp*ones(length(cenDist),1);
                newClassIdx(cenDist>mean(cenDist)+2*std(cenDist)) = bkgIdx;
                Grps3(Grps3==kGrp) = newClassIdx;
            end
        else
            % remove outliers from moving clusters
            for kGrp = 1:nClass
                if kGrp == bkgIdx
                    continue;
                end
                clusterTmp = pc_med(:, Grps3==kGrp);
                [idxTmp, distTmp] = knnsearch(clusterTmp', clusterTmp', 'k', 4);
                normDist = (sum(distTmp')/3)';
                newClassIdx = kGrp*ones(length(normDist),1);
                newClassIdx(normDist>mean(normDist)+3*std(normDist)) = bkgIdx;
                Grps3(Grps3==kGrp) = newClassIdx;
            end
        end
        Grps3d(:,nGrp3d) = Grps3;
    end
end

%% Display 3D Motion Segmentation Result
colors = distinguishable_colors(nClass); dim3d = 3;
for k = 1:size(Grps3d,2)
    traj3dColor = [];
    for i = 1:size(traj3d,1)/dim3d
        x = traj3d(dim3d*i-2:dim3d*i,:);
        for j = 1:nClass
            color = colors(j,:);
            x(:,Grps3d(:,k)==j) = repmat((color*255)',[1, sum(Grps3d(:,k)==j)]);
        end
        traj3dColor = [traj3dColor, floor(x)];
    end
    MSresultName3dPcd = ['MS3Dcolor_image_02_', paramDF.sequence(1:4),'_',num2str(paramDF.staSeq),...
        '_',num2str(paramDF.endSeq),'_',num2str(k),'.pcd'];
    savepcd(MSresultName3dPcd, traj3dColor);
end


%% Display Motion Segmentation 2D Projection Result
showMotSeg = 1;
proj3d = paramDF.traj2dAll(:, msRange);
if showMotSeg
    colors = distinguishable_colors(nClass); mrkSize = 10;
    for k = 1:size(Grps3d,2)
        figure(350+k);
        imshow(imgEnd);
        hold on;
        for i = 1:size(proj3d,1)/2
            for j = 1:nClass+1
                x = proj3d(2*i-1:2*i,:);
                if(j==nClass+1)
                    plot(x(1,Grps3d(:,k)==j), x(2,Grps3d(:,k)==j), 'og', 'Markersize', mrkSize);
                else
                    color = colors(j,:);
                    if (2*i == size(proj3d,1))
                        plot(x(1,Grps3d(:,k)==j), x(2,Grps3d(:,k)==j), 's', 'Color', color, 'Markersize', mrkSize);
                    else
                        plot(x(1,Grps3d(:,k)==j), x(2,Grps3d(:,k)==j), '.', 'Color', color, 'Markersize', mrkSize);
                    end
                end
            end
        end
    end
end

%% save result for 3d reconstruction
if 0
    trajTruck = [];
    for i = 1:size(traj3d,1)/3
        segMot = [];
        for k = 3%size(Grps3d,2)
            x = traj3d(3*i-2:3*i, :)*normNFrame/sqrt(3) + ...
                repmat(meanTraj3d,[1, size(traj3d,2)]);
            for j = 2
                %                 if(j== bkgIdx)
                segMot = [segMot, x(:,Grps3d(:,k)==j)];
                %                 end
                trajTruck = [trajTruck; x(:,Grps3d(:,k)==j)];
            end
        end
        %         idxString = sprintf('%03d',i+239);
        %         segMotName = ['FoV3d2d_image_02_0009_', idxString,'_features',num2str(j),'.pcd'];
        %         savepcd(segMotName, segMot);
    end
    save('trajTruck.mat', 'trajTruck');
end

%% Save results
%%
save(paramDF.saveTrkName, 'paramDF');


