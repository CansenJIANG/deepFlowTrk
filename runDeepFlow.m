addpath(genpath('/home/jiang/CvTools/deepmatching_1.2.2_c++'))
addpath(genpath('/home/jiang/CvTools/flow-code-matlab'))
deepMatchingExe = '/home/jiang/CvTools/deepmatching_1.2.2_c++/deepmatching ';
deepFlowExe = '/home/jiang/CvTools/DeepFlow_release2.0/deepflow2 ';
imgDir = '/home/jiang/CvDataset/KITTI/tracking_module/training/image_02/';
seqName = '0000/'; strFrame = 1; endFrame = 2;
im1Name = [imgDir, seqName, sprintf('%06d.png', strFrame)];
im2Name = [imgDir, seqName, sprintf('%06d.png', endFrame)];
floName = [im1Name(end-9:end-4),'_',im2Name(end-9:end-4),'.flo'];
im1 = im2single(imread(im1Name));
im2 = im2single(imread(im2Name));
% % % matches = deepmatching(im1, im2);
% % % load deep matching result
command = ['../deepmatching_1.2.2_c++/deepmatching /home/jiang/CvTools/deep_Matching_GUI/data/000001.png /home/jiang/CvTools/deep_Matching_GUI/data/000011.png -nt 0 -out matches.txt'];
[status,cmdout] = system(command);
load('matches.txt');
%exeDeepFlow = ['./deepflow2-static ', im1Name, ' ', im2Name, ];


deepFlowSet = [floName,' -match -kitti'];% -sintel , -middlebury
command = [deepMatchingExe,im1Name,' ',im2Name, ' | ',deepFlowExe, im1Name,' ', im2Name,' ',deepFlowSet];
% flow = deepflow2(im1, im2, matches(:,1:4), '-kitti'); % -sintel , -middlebury
tic;[status,cmdout] = system(command);toc;
flo = readFlowFile(floName);
figure, imshow(flowToColor(flo));
