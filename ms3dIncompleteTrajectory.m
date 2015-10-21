%% Motion Segmentation for Incomplete Trajectories
function [] = ms3dIncompleteTrajectory(paramDF)
zigma =25; nFeature =200; gndHeight = -1.3;
leftFileDir = [leftImgDir, sequence]; imgsLeft = dir( [leftFileDir,'*.png']);

% number of subspace
dim3d = 3; subspaceDim = 6;
MSresultName3D = [sequence(1:4),'_',num2str(startSeq), '_',num2str(endSeq),...
    'MS3D_L1ED_NoGnd_',num2str(nClass),'grps.mat'];
MSresultName2D = [sequence(1:4),'_',num2str(startSeq), '_',num2str(endSeq),...
    'MS2D_L1Noisy_NoGnd_',num2str(nClass),'grps.mat'];

% % Result name
% trajOFName = ['trajOF_',leftImgDir(end-8:end-1),'_',sequence(1:end-1),'_',...
%     num2str(startSeq),'_',num2str(endSeq),'.mat'];
% traj3DName = ['traj3D_',leftImgDir(end-8:end-1),'_',sequence(1:end-1),'_',...
%     num2str(startSeq),'_',num2str(endSeq),'.mat'];
% traj3DprojName = ['traj3Dproj_',leftImgDir(end-8:end-1),'_',sequence(1:end-1),'_',...
%     num2str(startSeq),'_',num2str(endSeq),'.mat'];
% lostTraj3DName = ['lostTraj3D_image_02_', sequence(1:4),'_',num2str(startSeq),...
%     '_',num2str(endSeq),'.mat'];
% traj3dValidPcdName = ['validTraj3D_image_02_', sequence(1:4),'_',num2str(startSeq),...
%     '_',num2str(endSeq),'.pcd'];
% traj3dSegColorPcdName = ['traj3DSegColor_image_02_', sequence(1:4),'_',num2str(startSeq),...
%     '_',num2str(endSeq),'.pcd'];
% 
% traj3DValidName = ['traj3DValid_',leftImgDir(end-8:end-1),'_',sequence(1:end-1),'_',...
%     num2str(startSeq),'_',num2str(endSeq),'.mat'];
% traj3DValidprojName = ['traj3DValidproj_',leftImgDir(end-8:end-1),'_',sequence(1:end-1),'_',...
%     num2str(startSeq),'_',num2str(endSeq),'.mat'];
% 
% load(traj3DName); load(traj3DprojName);load(lostTraj3DName);load(trajOFName);
showFig = 0;
traj3d = []; traj3dProj = []; showFig = 0; gndIdx = []; 
if showFig
    figure(333); hold on; view(3); grid on;
end
for i = 1:length(traj3D)
    tmpTraj3D = traj3D{i};
    tmpTraj3D(:,lostIdx) = [];
    tmpProjTraj3D = traj3Dproj{i};
    tmpProjTraj3D(:,lostIdx) = [];
    gndIdx = [gndIdx, find(tmpTraj3D(end,:)<= gndHeight)];
    gndIdx = unique(gndIdx);
    if showFig
        if mod(i,3) ==1
            plot3(tmpTraj3D(1,:),tmpTraj3D(2,:),tmpTraj3D(3,:),'.r');
        elseif mod(i,3) ==2
            plot3(tmpTraj3D(1,:),tmpTraj3D(2,:),tmpTraj3D(3,:),'.g');
        else
            plot3(tmpTraj3D(1,:),tmpTraj3D(2,:),tmpTraj3D(3,:),'.b');
        end
    end
    traj3d = [traj3d;tmpTraj3D];
    traj3dProj = [traj3dProj; tmpProjTraj3D];
end
% Remove ground points
traj3d(:,gndIdx) = []; traj3dProj(:,gndIdx) = [];
trajOFframe0 = trajOF{1}'; 
trajOFframe0(:, lostIdx) = []; trajOFframe0(:, gndIdx) = []; 

%% normalize data
meanTraj3d = mean(traj3d,2); 
meanTraj3d = mean(reshape(meanTraj3d,[3, length(meanTraj3d)/3]),2);
traj3d = traj3d - repmat(meanTraj3d, [size(traj3d,1)/3, size(traj3d,2)]);
normNFrame = []; dim = 3;traj3dAllSeq=[];
for i = 1:size(traj3d,1)/dim
    traj1Frame = traj3d(dim*i-dim+1:dim*i, :);
    normNFrame = [normNFrame; sqrt(sum(traj1Frame.^2,1))];
    traj3dAllSeq = [traj3dAllSeq, traj1Frame];
end
normNFrame = sum(normNFrame(:))/size(normNFrame,1)/size(normNFrame,2);
traj3d = traj3d*sqrt(3)/normNFrame;

traj3dAllSeq = traj3dAllSeq+repmat(meanTraj3d,[1, size(traj3dAllSeq,2)]);
savepcd('traj3dAllSeq.pcd', traj3dAllSeq);
%% remove outlier trajectories
traj3d_shifted = circshift(traj3d, [-3, 0]);
trajDiff = traj3d - traj3d_shifted;
lenTraj3d = length(traj3d);

while(length(traj3d)>2*nFeature&&length(traj3d)>0.5*lenTraj3d)
    normDiff = [];
    for i = 1:size(trajDiff,2)
        tmpXYZ = trajDiff(:, i); tmpXYZ = reshape(tmpXYZ, [dim3d, length(tmpXYZ)/dim3d ])';
        tmpXYZ(1,:) = 0; tmpXYZ(end,:) = 0;
        normDiff = [normDiff, mean( sqrt(sum(tmpXYZ.*tmpXYZ,2)))];
    end
    inlierIdx = normDiff<median(normDiff);
    traj3d = traj3d(:, inlierIdx);
    traj3dProj = traj3dProj(:, inlierIdx);
    trajDiff = trajDiff(:, inlierIdx);
    trajOFframe0 = trajOFframe0(:, inlierIdx);
end
traj3dLeft = []; traj2dValid = [];
for i = 1:size(traj3d,1)/dim3d
    X3d = traj3d(dim3d*i-dim3d+1:dim3d*i,:)*normNFrame/sqrt(3) + ...
        repmat(meanTraj3d,[1, size(traj3d,2)]);
    traj3dLeft = [traj3dLeft, X3d];
    X2d = traj3dProj(2*i-1:2*i,:);
    traj2dValid = [traj2dValid; X2d];
end
% showPointCloud(traj3dLeft');
% savepcd('traj3dLeft.pcd', traj3dLeft);
%% Trajectories downsampling
if 1
mdnOF = [median(trajOFframe0(1,:)), median(trajOFframe0(2,:))];
trajOFframe0 = trajOFframe0 - repmat(mdnOF', [1, size(trajOFframe0,2)]);
trajOFframe0 = sqrt(sum(trajOFframe0.*trajOFframe0));

inlierIdx = trajOFframe0<=600;
trajOFframe0 = trajOFframe0(inlierIdx);
traj3d = traj3d(:, inlierIdx);
traj3dProj = traj3dProj(:, inlierIdx);
trajDiff = trajDiff(:, inlierIdx);

trajOFframe0 = exp(trajOFframe0/zigma);
meanOF = median(trajOFframe0); stdvOF = std(trajOFframe0);
trajOFframe0(trajOFframe0>(meanOF+3*stdvOF)) = [];


trajOFframe0 = trajOFframe0/sum(trajOFframe0);
OFcmf = trajOFframe0(1);
for i = 2:length(trajOFframe0)
    OFcmf(i) = OFcmf(i-1)+trajOFframe0(i);
end
figure; plot(OFcmf,'-.'); grid on;

sampleIdx = []; smpfactor = 1.0;
if 0
    while length(sampleIdx)< nFeature
        % generate 10% more samples
        sampleVal = rand( floor( smpfactor*(nFeature-length(sampleIdx))),1);
        for i = 1:length(sampleVal)
            idxTmp = bisectionSearch(sampleVal(i), OFcmf);
            sampleIdx = [sampleIdx, idxTmp];
        end
        sampleIdx = unique(sampleIdx);
        if length(sampleIdx) < 0.8*nFeature
            smpfactor = 3;
        end
    end
else
    sampleVal = [0:1/nFeature:1];
    for i = 1:length(sampleVal)
        idxTmp = bisectionSearch(sampleVal(i), OFcmf);
        sampleIdx = [sampleIdx, idxTmp];
    end
    sampleIdx = unique(sampleIdx);
end
if nFeature > length(sampleIdx); nFeature = length(sampleIdx); end
traj2dValid = traj2dValid(:,sampleIdx(1:nFeature));
traj3d = traj3d(:,sampleIdx(1:nFeature));
traj3dValidPcd = [];
for i = 1:size(traj3d,1)/dim3d
    X3d = traj3d(dim3d*i-dim3d+1:dim3d*i,:)*normNFrame/sqrt(3) + ...
        repmat(meanTraj3d,[1, size(traj3d,2)]);
    traj3dValidPcd = [traj3dValidPcd, X3d];
end
savepcd(traj3dValidPcdName, traj3dValidPcd);
else
    sampleIdx = [];
    while length(sampleIdx)<nFeature
        a = 1;
        b = length(traj3d);
        sampleIdx = [sampleIdx; floor((b-a).*rand(nFeature,1) + a)];
        sampleIdx = unique(sampleIdx);
    end
    traj2dValid = traj2dValid(:,sampleIdx(1:nFeature));
    traj3d = traj3d(:,sampleIdx(1:nFeature));
end

% load nobj_setting.mat;

%% show 2D trajectories
imgName1 = [leftFileDir,imgsLeft(startSeq+1).name];
img1 = imread(imgName1); figure(661), clf; imshow(img1); hold on;
trajCol = size(traj2dValid,2);
trajRow = size(traj2dValid,1);
drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),2);  

 
for j = 1:trajCol
%     xo = [traj2dValid(end-3, j), traj2dValid(end-1, j)];
%     yo = [traj2dValid(end-2, j), traj2dValid(end, j)];
%     h = drawArrow(xo,yo);
%     h.Color = 'red'; h.MaxHeadSize = 20; h.AutoScale = 'on'; 
%     h.AutoScaleFactor = 2;
%     h.LineWidth = 1.5; 
    plot(traj2dValid(1:2:trajRow,j), traj2dValid(2:2:trajRow,j), '-r', 'Linewidth', 1);
end
hold off;
imgName2 = [leftFileDir,imgsLeft(endSeq).name];
img2 = imread(imgName2); figure(662), imshow(img2); hold on;
for j = 1:trajCol
    plot(traj2dValid(1,j), traj2dValid(2,j), 'og', 'Markersize', 5, 'Linewidth', 1.5);
    plot(traj2dValid(1:2:trajRow,j), traj2dValid(2:2:trajRow,j), '-r', 'Linewidth', 1);
end
hold off;
% save( 'system_setting.mat');
clear; close all; clc; load system_setting.mat;
%% Subtract first frame
traj3dNo1stFrame = traj3d - repmat(traj3d(1:3,:), [size(traj3d,1)/3, 1]);
traj3dNo1stFrame(1:3,:) = []; % traj3d = traj3dNo1stFrame;
traj3dNo1stFrame = traj3d;
traj2dNo1stFrame = traj2dValid - repmat(traj2dValid(1:2,:), [size(traj2dValid,1)/2, 1]);
traj2dNo1stFrame(1:2,:) = []; %traj2dValid = traj2dNo1stFrame;
traj2dNo1stFrame = traj2dValid;

%% Sparce Spectural Clustering
resultName = dir(MSresultName3D);
if ~isempty(resultName)
    load( resultName.name );
    load(traj3DValidName);
    load(traj3DValidprojName);
    traj2dValid;
else
    tic;
    %resultName = [dataName(1:end-4),'_L1ED_',num2str(n),'grps.mat'];
    Cst3d = 0; %Enter 1 to use the additional affine constraint sum(c) == 1
    OptM3d = 'L1ED'; %OptM can be {'L1Perfect','L1Noisy','Lasso','L1ED'}
    lambda = 0.001; %Regularization parameter in 'Lasso' or the noise level for 'L1Noise'
    
%    subspaceDim = 3; % dimension of subspace
    CMat3d = SparseCoefRecovery(traj3dNo1stFrame,Cst3d,OptM3d,lambda);
    
    CKSym3d = BuildAdjacency(CMat3d,subspaceDim);
%     outlierThd = 0.02;
%     affinityMask = ones(nFeature, nFeature);
%     affinityMask(CKSym<=outlierThd) = 0;
%     cnt = sum(affinityMask,1);
%     cntIdx = find(cnt<3);
%     CKSym(cntIdx,:) = []; CKSym(:,cntIdx) = [];
    %CKSym(cntIdx,:); CKSym = CKSym(:, cntIdx);
%     cnt(cnt<2) = 0;  cnt(cnt==0) = 1; 
%     CKSym(cnt,:) = 0.000001; CKSym(:, cnt) = 0.000001; %CKSym = CKSym + 0.001*eye(size(CKSym));
    [Grps3d , SingVals3d, LapKernel3d] = SpectralClustering(CKSym3d,nClass);
    toc;
    % Missrate = Misclassification(Grps,GT_rand);
    % display('Miss classification rate: '), disp(Missrate')
%     CKSym_Kmean = CKSym.*triu(ones(size(CKSym)));
%     for i = 1:3
%         idx = 1:nFeature;
%         sortedIdx = [idx', Grps(:,i)];
%         sortedIdx = sortrows(sortedIdx, 2);
%     end
%     for i=1:length(cntIdx)
%         Grps()
%         
%     end
% for i = 1:3
%     Vector=Grps(:,i);  
%     c=false(1,length(Vector)+length(cntIdx));
%     c(cntIdx)=true;
%     result=nan(size(c));
%     result(~c)=Vector;
%     result(c)=nClass+1;
%     GrpsN(:,i) = result';
% end
%     Grps = GrpsN;

    save(MSresultName3D, 'Grps3d');
end

%% regrouping based on 3d distance
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

    if 1
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

%% Display 3D Motion Segmentation Result
colors = distinguishable_colors(nClass);
for k = 1:size(Grps3d,2)
    traj3dColor = [];
    for i = 1:size(traj3d,1)/dim3d
        x = traj3d(dim3d*i-dim3d+1:dim3d*i,:);
        for j = 1:nClass
            color = colors(j,:);
            x(:,Grps3d(:,k)==j) = repmat((color*255)',[1, sum(Grps3d(:,k)==j)]);
        end
        traj3dColor = [traj3dColor, floor(x)];
    end
    MSresultName3dPcd = ['MS3Dcolor_image_02_', sequence(1:4),'_',num2str(startSeq),...
    '_',num2str(endSeq),'_',num2str(k),'.pcd'];
    savepcd(MSresultName3dPcd, traj3dColor);
    save(traj3DValidName, 'traj3d');
    save(traj3DValidprojName,'traj2dValid');
end



%% Display Motion Segmentation 2D Projection Result
showMotSeg = 1;
proj3d = traj2dValid;
if showMotSeg
    colors = distinguishable_colors(nClass); mrkSize = 10;
    for k = 1:size(Grps3d,2)
        figure(350+k);
        imshow(img2);
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

figure;imshow(img2);
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
%% save motion segmentation results saperately
if 0
for i = 1:size(traj3d,1)/3
    segMot = [];
    for k = size(Grps3d,2)
        x = traj3d(3*i-2:3*i, :)*normNFrame/sqrt(3) + ...
            repmat(meanTraj3d,[1, size(traj3d,2)]);
        for j = 2
            segMot = [segMot, x(:,Grps3d(:,k)==j)];
        end
    end
    idxString = sprintf('%03d',i+584);
    segMotName = ['FoV3d2d_image_02_0019_', idxString,'_features.pcd'];
    savepcd(segMotName, segMot);
end

    Grp3 = Grps(:, 3);
    trajMot = traj3d(:, Grp3~=bkgIdx);
    trajMotName = ['FoV3d2d_image_02_0000_',num2str(startSeq),'_',num2str(endSeq),'_trajMot.mat'];
    save(trajMotName, 'trajMot');
    
    traj3dBkg = []; step = size(traj3dLeft,2)/size(traj3d,1)*3;
    for i = 1:size(traj3d,1)/3
        traj3dBkg = [traj3dBkg; traj3dLeft(:, (i-1)*step+1:i*step)];
    end
    
    for i = 1:size(traj3d,1)/3
    segMot = [];
    x = traj3dBkg(3*i-2:3*i, :);

    idxString = sprintf('%02d',i+239);
    segMotName = ['FoV3d2d_image_02_0009_', idxString,'_bkg.pcd'];
    savepcd(segMotName, x);
end
    
end

%% Motion Segmentation in 2D
K19 = [718.335100000000,0,600.389100000000;
    0,718.335100000000,181.512200000000;
    0,0,1];
K20 = [718.856000000000,0,607.192800000000;
    0,718.856000000000,185.215700000000;
    0,0,1];
K = K19;
traj2d = traj2dValid;
for i = 1:size(traj2d,1)/2
    X = [traj2d(2*i-1:2*i,:);ones(1,size(traj2d,2))];
    x = inv(K)*X;
    traj2d(2*i-1:2*i,:) = x(1:2, :);
end
%% Sparce Spectural Clustering 2D
resultName = dir(MSresultName2D);
if ~isempty(resultName)
    load(resultName.name );
    load(traj3DValidprojName);
else
    tic;
    Cst2d = 1; %Enter 1 to use the additional affine constraint sum(c) == 1
    OptM2d = 'Lasso'; %OptM can be {'L1Perfect','L1Noisy','Lasso','L1ED'}
    lambda = 0.001; %Regularization parameter in 'Lasso' or the noise level for 'L1Noise'
    k = 4; %dimemsion of subspace;
    CMat2d = SparseCoefRecovery(traj2dNo1stFrame,Cst2d,OptM2d,lambda);
    
    CKSym2d = BuildAdjacency(CMat2d,k);
    [Grps2d , SingVals2d, LapKernel2d] = SpectralClustering(CKSym2d,nClass);
    % Missrate = missClassificationRate(Grps,GT_rand, Traj2D);
    % Missrate = [Missrate; mean( Misclassification(Grps,GT_rand) )];
    % display('Miss classification rate: '), disp(Missrate')
    toc;
    save(MSresultName2D, 'Grps2d');
    save(traj3DValidprojName,'traj2d');
end
%% Display Motion Segmentation Result
colors = distinguishable_colors(nClass);
for k = 1:size(Grps2d,2)
    figure(110+k),hold on;axis([-1 1 -1 1])
    for i = 1:size(traj2d,1)/2
        for j = 1:nClass
            x = traj2d(2*i-1:2*i,:);
            color = colors(j,:);
            plot(x(1,Grps2d(:,k)==j), x(2,Grps2d(:,k)==j), '+', 'Color', color);
        end
    end
end

%% Display Motion Segmentation Result
showMotSeg = 1; traj2dImg = [];
for i = 1:size(traj2d,1)/2
    traj2dTmp = K*[traj2d(i*2-1:2*i, :); ones(1, size(traj2d,2))];
    traj2dImg = [traj2dImg; traj2dTmp(1:2,:)];
end

if showMotSeg
    colors = distinguishable_colors(nClass); mrkSize = 10;
%     colors = [1,0,0;1,0,0;0,0,1];
    for k = 1:size(Grps2d,2)
        figure(120+k);
        imshow(img2);
        hold on;
        for i = 1:size(traj2dImg,1)/2
            for j = 1:nClass
                x = traj2dImg(2*i-1:2*i,:);
                color = colors(j,:);
                if (2*i == size(traj2dImg,1))
%                         xo = [traj2dValid(end-3, j), traj2dValid(end-1, j)];
%                     yo = [traj2dValid(end-2, j), traj2dValid(end, j)];
%                     h = drawArrow(xo,yo);
%                     h.Color = color; h.MaxHeadSize = 20; h.AutoScale = 'on'; 
%                     h.AutoScaleFactor = 2;
%                     h.LineWidth = 1.5; 
                    plot(x(1,Grps2d(:,k)==j), x(2,Grps2d(:,k)==j), 's', 'Color', color, 'Markersize', mrkSize);
                else
                    plot(x(1,Grps2d(:,k)==j), x(2,Grps2d(:,k)==j), '.', 'Color', color, 'Markersize', mrkSize);
                end
            end
        end
    end
end
end
