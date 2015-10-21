%% Sequential Optical Flow Tracking
%% Option 1: forward optical flow tracking
%% Option 2： backward optical flow tracking
function [paramDF] = SequentialOpticalFlowTrking(paramDF, opt, nFrm)
if nargin <3
    nFrm = 6;
else
    nFrm = nFrm + 1;
end

if nargin <2 | opt== 1
    opt = 1; 
    stepsDirection = 1;
    endSeq = paramDF.endSeq;
    staSeq = paramDF.staSeq;
end

if opt==2
    stepsDirection = -1;
    endSeq = paramDF.staSeq;
    staSeq = paramDF.endSeq;
elseif opt==3
    stepsDirection = -1;
    staSeq = paramDF.staSeq;
    endSeq = paramDF.staSeq-nFrm;
elseif opt==4
    stepsDirection = 1;
    staSeq = paramDF.endSeq;
    endSeq = paramDF.endSeq+nFrm;
end

%% Starting Frame to track
% load velodyne points
fid  = fopen(sprintf('%s/tracking_module/training/velodyne/%04s/%06d.bin',...
     paramDF.base_dir, paramDF.sequence(1:end-1), staSeq-1),'rb');
velo = fread(fid,[4 inf],'single')';
fclose(fid);

idx = velo(:,1)<0; velo(idx,:) = []; % remove all points behind image plane
idx = velo(:,1)>paramDF.distThd; velo(idx,:) = []; % remove all points father than 15ms
% project to image plane (exclude luminance)
[velo_img, velo_depth] = project(velo(:,1:3),paramDF.P_velo_to_img); velo_img = velo_img';

% Lets validate only those 3D and projected points which are within image
% Image size
Ix = 1238; Iy = 374; traj3Dproj = []; traj3D = []; trajOF = [];
ptList = velo_img(1,:)>0.5 & velo_img(2,:)>0.5 &...
    velo_img(1,:)< (Ix- 0.5) & velo_img(2,:)<(Iy- 0.5);

pts3D   = velo(ptList,1:3)'; velo_depth = velo_depth(ptList);
proj3D  = velo_img(:,ptList);
proj3D(1,proj3D(1, :)<1) = 1; proj3D(2,proj3D(2, :)<1) = 1;
traj3Dproj{length(traj3Dproj)+1} = proj3D;
traj3D{length(traj3D)+1} = pts3D;

%% Compute Dense Optical Flow
leftImgRef = imread([paramDF.leftFileDir,paramDF.imgsLeft(paramDF.staSeq).name]);
% rightImgRef = rgb2gray(imread([rightFileDir,imgsRight(staSeq).name]));

optiFlowSeq = []; showFig = 1; 
for methodOpt = 1
    lostIdx = [];
    for idxframe = staSeq:stepsDirection:endSeq-stepsDirection

        imgName1 = [paramDF.leftFileDir, paramDF.imgsLeft(idxframe).name];
        imgName2 = [paramDF.leftFileDir, paramDF.imgsLeft(idxframe+stepsDirection).name];
        
        floName = [paramDF.sequence(1:end-1),'_',imgName1(end-9:end-4),'_',imgName2(end-9:end-4),'.flo'];
        im1 = im2single(imread(imgName1));
        im2 = im2single(imread(imgName2));
        deepFlowSet = [floName,' -match -kitti'];% -sintel , -middlebury
        command = [paramDF.deepMatchingExe,imgName1,' ',imgName2, ' | ', ...
            paramDF.deepFlowExe, imgName1,' ', imgName2,' ',deepFlowSet];
        % tic;[status,cmdout] = system(command);toc;
        % flo = readFlowFile(floName);
        % figure, imshow(flowToColor(flo));
        
        % this is the core part of calling the mexed dll file for computing optical flow
        % it also returns the time that is needed for two-frame estimation
        if exist(floName)
            flo = readFlowFile(floName);
            vx = flo(:,:,1); vy = flo(:,:,2);
        else
            tic;
            [status,cmdout] = system(command);
            flo = readFlowFile(floName);
            vx = flo(:,:,1); vy = flo(:,:,2);
            toc
%             optiFlowSeq{steps*(idxframe+1-staSeq), 1} = vx;
%             optiFlowSeq{steps*(idxframe+1-staSeq), 2} = vy;
        end
        %% Project 3D to 2D
        fid  = fopen(sprintf('%s/tracking_module/training/velodyne/%04s/%06d.bin',...
            paramDF.base_dir, paramDF.sequence(1:end-1),idxframe-1+stepsDirection),'rb');
        velo = fread(fid,[4 inf],'single')';
        fclose(fid);
        
        idx = velo(:,1)<0; velo(idx,:) = []; % remove all points behind image plane
        idx = velo(:,1)>paramDF.distThd; velo(idx,:) = []; % remove all points father than 15ms
        
        % project to image plane (exclude luminance)
        [velo_img, velo_depth] = project(velo(:,1:3),paramDF.P_velo_to_img); velo_img = velo_img';
        
        % Lets validate only those 3D and projected points which are within image
        ptList = velo_img(1,:)>0.5 & velo_img(2,:)>0.5 &...
            velo_img(1,:)< (Ix- 0.5) & velo_img(2,:)<(Iy- 0.5);
        
        pts3Dtrk  = velo(ptList,1:3)'; velo_depth = velo_depth(ptList);
        proj3Dtrk  = velo_img(:,ptList);
        
        
        %% Compute OF for 2D projections on img1
        trkFeat = proj3D'; trkOF = zeros(size(trkFeat));
        for i = 1:length(trkFeat)
            px = floor(trkFeat(i,1));
            py = floor(trkFeat(i,2));
            if(trkFeat(i,1)==px && trkFeat(i,2)==py)
                trkFeat(i,1) = trkFeat(i,1)+vx(py, px);
                trkFeat(i,2) = trkFeat(i,2)+vy(py, px);
                lostIdx = [lostIdx; i];
            else
                if(px<1 || px>size(im2,2)-1 ||py<1 || py>size(im2,1)-1)
                    trkFeat(i,:) = [1, 1];
                    continue;
                end
                Neigh4 = [px, py; px+1, py; px, py+1; px+1, py+1]; % 4 neighbourhood
                
                for j = 1:4
                    Neigh4vx(j) = vx(Neigh4(j,2), Neigh4(j,1));
                    Neigh4vy(j) = vy(Neigh4(j,2), Neigh4(j,1));
                end
                weight4 = repmat(trkFeat(i,:),[4, 1]) - Neigh4;
                weight4 = weight4.*weight4;
                weight4 = sum(weight4'); weight4 = 1./weight4;
                deltaX = weight4*Neigh4vx'/sum(weight4);
                deltaY = weight4*Neigh4vy'/sum(weight4);
                trkFeat(i,1) = trkFeat(i,1) + deltaX;
                trkFeat(i,2) = trkFeat(i,2) + deltaY;
                trkOF(i,1)  = deltaX;
                trkOF(i,2)  = deltaY;
            end
            if(trkFeat(i,1)<1 || trkFeat(i,1)>size(im2,2)-1 ||...
                    trkFeat(i,2)<1 || trkFeat(i,2)>size(im2,1)-1)
                trkFeat(i,:) = [1, 1];
                lostIdx = [lostIdx; i];
                continue;
            end
        end
        
        if showFig
            figure(1), imshow(im1);hold on;
            plot(proj3D(1,:), proj3D(2,:),'.r'); % original projected points
                    plot(trkFeat(:,1), trkFeat(:,2),'.g'); % with optical flow
            
            figure(2), imshow(im2); hold on;
            plot(trkFeat(:,1),trkFeat(:,2), '.','color',[0, 1, 0]);
            %             plot(proj3Dtrk(1,:),proj3Dtrk(2,:), '.b');
        end
        %% Find tracked points from 2D projection of next 3d cloud
        [knnIdx, knnDist] = knnsearch(proj3Dtrk', trkFeat);
        pts3Dcorr = pts3Dtrk(:, knnIdx); pts3Dcorr(:, lostIdx) = 0;
        traj3D{length(traj3D)+1} = pts3Dcorr;
        ptsProj3Dcorr = proj3Dtrk(:, knnIdx); ptsProj3Dcorr(:, lostIdx) = 1;
        traj3Dproj{length(traj3Dproj)+1} = ptsProj3Dcorr;
        trajOF{length(trajOF)+1} = trkOF;
        
        %% Reset first frame to trk
        trkFeat = proj3Dtrk(:, knnIdx)';
        trkFeat(lostIdx, :) = ones(length(lostIdx),2);
        pts3D = pts3Dtrk(:, knnIdx);
        pts3D(:, lostIdx) = zeros(length(lostIdx),3)';
        proj3D = proj3Dtrk(:, knnIdx);
        proj3D(:,lostIdx) = ones(length(lostIdx),2)';
        lostIdx = unique(lostIdx);
        
        if showFig
            figure(3), imshow(im2); hold on;
            plot(proj3D(1,:),proj3D(2,:), '.','color',[0, 1, 0]);
        end
        
        if (idxframe == endSeq -1)
            optiFlowSeqName = ['OF_',paramDF.leftImgDir(end-8:end-1),'_',paramDF.sequence(1:end-1),'_',...
                num2str(staSeq),'_',num2str(endSeq),'_DF.mat'];
%             save(optiFlowSeqName, 'optiFlowSeq');
            optiFlowSeqName = '_';            
%             optiFlowSeq = [];
        end
    end
%     save(paramDF.traj3DName, 'traj3D');
%     save(paramDF.traj3DprojName, 'traj3Dproj');
%     save(paramDF.lostTraj3DName, 'lostIdx');
    idxframe = 1;
    if(opt==1)
        ForwardTraj3D = traj3D; paramDF.ForwardTraj3D = ForwardTraj3D;
        ForwardTraj3Dproj = traj3Dproj; paramDF.ForwardTraj3Dproj = ForwardTraj3Dproj;
        ForwardLostIdx = lostIdx; paramDF.ForwardLostIdx = ForwardLostIdx;
    elseif(opt==2)
        BackwardTraj3D = traj3D; paramDF.BackwardTraj3D = BackwardTraj3D;
        BackwardTraj3Dproj = traj3Dproj; paramDF.BackwardTraj3Dproj = BackwardTraj3Dproj;
        BackwardLostIdx = lostIdx; paramDF.BackwardLostIdx = BackwardLostIdx;
    elseif(opt==3)
        BackwardTraj3D = traj3D; paramDF.nFrmBwdTraj3D = BackwardTraj3D;
        BackwardTraj3Dproj = traj3Dproj; paramDF.nFrmBwdTraj3Dproj = BackwardTraj3Dproj;
        BackwardLostIdx = lostIdx; paramDF.nFrmBwdLostIdx = BackwardLostIdx;      
    elseif(opt==4)
        ForwardTraj3D = traj3D; paramDF.nFrmFwdTraj3D = ForwardTraj3D;
        ForwardTraj3Dproj = traj3Dproj; paramDF.nFrmFwdTraj3Dproj = ForwardTraj3Dproj;
        ForwardLostIdx = lostIdx; paramDF.nFrmFwdLostIdx = ForwardLostIdx;
    end
end

end