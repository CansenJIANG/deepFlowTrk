%% =========================================================================
clear; close all;clc;
paramDF.leftImgDir  = '/home/jiang/CvDataset/KITTI/tracking_module/training/image_02/';
paramDF.rightImgDir = '/home/jiang/CvDataset/KITTI/tracking_module/training/image_03/';
paramDF.deepMatchingExe = '/home/jiang/CvTools/deepmatching_1.2.2_c++/deepmatching ';
paramDF.deepFlowExe     = '/home/jiang/CvTools/DeepFlow_release2.0/deepflow2 ';
sequence = '0001/'; staSeq =372; endSeq = 394;

paramDF.leftFileDir = [paramDF.leftImgDir, sequence];
paramDF.imgsLeft    = dir([paramDF.leftFileDir,'*.png']);
paramDF.rightFileDir= [paramDF.rightImgDir, sequence];
paramDF.imgsRight   = dir([paramDF.rightFileDir,'*.png']);

stepsDirection = 1;

for idxframe = staSeq:stepsDirection:endSeq
    imgName1 = [paramDF.leftFileDir, paramDF.imgsLeft(idxframe).name];
    imgName2 = [paramDF.leftFileDir, paramDF.imgsLeft(idxframe+stepsDirection).name];
    
    floName = [sequence(1:end-1),'_',imgName1(end-9:end-4),'_',imgName2(end-9:end-4),'.flo'];
    im1 = im2single(imread(imgName1));
    im2 = im2single(imread(imgName2));
    deepFlowSet = [floName,' -match -kitti'];% -sintel , -middlebury
    command = [paramDF.deepMatchingExe,imgName1,' ',imgName2, ' | ', ...
        paramDF.deepFlowExe, imgName1,' ', imgName2,' ',deepFlowSet];
    if ~exist(floName)
        [status,cmdout] = system(command);
    end
    
    floName = [sequence(1:end-1),'_',imgName2(end-9:end-4),'_',imgName1(end-9:end-4),'.flo'];
    deepFlowSet = [floName,' -match -kitti'];
    command = [paramDF.deepMatchingExe,imgName2,' ',imgName1, ' | ', ...
        paramDF.deepFlowExe, imgName2,' ', imgName1,' ',deepFlowSet];
    if ~exist(floName)
        [status,cmdout] = system(command);
    end
end
