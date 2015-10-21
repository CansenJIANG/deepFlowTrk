%% function [idx] = flowSampling(OF, candidatePos, smpNb)
function [inlierIdx, sampleIdx, bigDiffIdx] = flowSampling(traj3d, traj3dProj, flo, nFeature)
dim3d = 3; zigma = 85;
trajOF = interpolateOF(traj3dProj(1:2,:), flo)';

lenTrajColumn = [];
for i = 1:size(traj3d, 2)
    tmpXYZ = traj3d(:, i); 
    tmpXYZ = reshape(tmpXYZ, [dim3d, length(tmpXYZ)/dim3d ])';
    lenTrajColumn(i) = length(find(tmpXYZ(:,1)==0));
end
%% normalize data
meanTraj3d = mean(traj3d,2);
meanTraj3d = mean(reshape(meanTraj3d,[3, length(meanTraj3d)/3]),2);
traj3d = traj3d - repmat(meanTraj3d, [size(traj3d,1)/3, size(traj3d,2)]);
normNFrame = []; dim = 3; traj3dAllSeq=[];
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
trajDiff  = traj3d - traj3d_shifted;
lenTraj3d = length(traj3d);
normDiff  = [];

% for i = 1:size(trajDiff, 2)
%     tmpXYZ = trajDiff(:, i); tmpXYZ = reshape(tmpXYZ, [dim3d, length(tmpXYZ)/dim3d ])';
%     tmpXYZ(1,:) = 0; tmpXYZ(end,:) = 0;
%     normDiff = [normDiff, sum(sqrt(sum(tmpXYZ.*tmpXYZ,2)))/(size(trajDiff,1)/dim3d - lenTrajColumn(i))];
% %     normDiff = [normDiff, mean(sqrt(sum(tmpXYZ.*tmpXYZ,2)))];
% end
for i = 1:size(trajDiff, 2)
    tmpXYZ = trajDiff(:, i); tmpXYZ = reshape(tmpXYZ, [dim3d, length(tmpXYZ)/dim3d ])';
    tmpXYZ = tmpXYZ(1:length(tmpXYZ) - lenTrajColumn(i),:);
    tmpXYZ = tmpXYZ(2:end-1,:); 
    normDiff = [normDiff, sum(sqrt(sum(tmpXYZ.*tmpXYZ,2)))/(size(trajDiff,1)/dim3d - lenTrajColumn(i))];
%     normDiff = [normDiff, mean(sqrt(sum(tmpXYZ.*tmpXYZ,2)))];
end
inlierNb = floor(min(max(2*nFeature, 0.5*lenTraj3d), 2000));
if inlierNb<500
    inlierNb = length(traj3d);
end
sortedNormDiff = sort(normDiff); bigDiffIdx = find(normDiff>sortedNormDiff(end-50));
inlierIdx = find(normDiff<sortedNormDiff(inlierNb));
traj3d = traj3d(:, inlierIdx);
traj3dProj = traj3dProj(:, inlierIdx);
trajDiff = trajDiff(:, inlierIdx);
trajOF= trajOF(:, inlierIdx);

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

mdnOF = [median(trajOF(1,:)), median(trajOF(2,:))];
trajOF = trajOF - repmat(mdnOF', [1, size(trajOF,2)]);
trajOF = sqrt(sum(trajOF.*trajOF));

% inlierIdx = trajOF<=600;
% trajOF = trajOF(inlierIdx);
% traj3d = traj3d(:, inlierIdx);
% traj3dProj = traj3dProj(:, inlierIdx);
% trajDiff = trajDiff(:, inlierIdx);

trajOF = exp(-trajOF/zigma);
meanOF = median(trajOF); stdvOF = std(trajOF);
trajOF(trajOF>(meanOF+3*stdvOF)) = [];


trajOF = trajOF/sum(trajOF);
OFcmf = trajOF(1);
for i = 2:length(trajOF)
    OFcmf(i) = OFcmf(i-1)+trajOF(i);
end
figure; plot(OFcmf,'-.'); grid on;

sampleIdx = [];
sampleVal = [0:1/nFeature:1];
for i = 1:length(sampleVal)
    idxTmp = bisectionSearch(sampleVal(i), OFcmf);
    sampleIdx = [sampleIdx, idxTmp];
end
sampleIdx = unique(sampleIdx);

if nFeature > length(sampleIdx); nFeature = length(sampleIdx); end
traj2dValid = traj2dValid(:,sampleIdx(1:nFeature));
traj3d = traj3d(:,sampleIdx(1:nFeature));
traj3dValidPcd = [];
for i = 1:size(traj3d,1)/dim3d
    X3d = traj3d(dim3d*i-dim3d+1:dim3d*i,:)*normNFrame/sqrt(3) + ...
        repmat(meanTraj3d,[1, size(traj3d,2)]);
    traj3dValidPcd = [traj3dValidPcd, X3d];
end

sampleIdx = inlierIdx(sampleIdx(1:nFeature));
end