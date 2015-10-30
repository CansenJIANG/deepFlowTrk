%% Deep Flow Setting
mode = 'testing';
paramDF.calib_dir = ['/home/jiang/CvDataset/KITTI/tracking_module/',mode,'/calib/'];
paramDF.base_dir  = '/home/jiang/CvDataset/KITTI';
paramDF.imgDir    = ['/home/jiang/CvDataset/KITTI/tracking_module/',mode,'/image_02/'];
paramDF.leftImgDir  = ['/home/jiang/CvDataset/KITTI/tracking_module/',mode,'/image_02/'];
paramDF.rightImgDir = ['/home/jiang/CvDataset/KITTI/tracking_module/',mode,'/image_03/'];
paramDF.deepMatchingExe = '/home/jiang/CvTools/deepmatching_1.2.2_c++/deepmatching ';
paramDF.deepFlowExe     = '/home/jiang/CvTools/DeepFlow_release2.0/deepflow2 ';
paramDF.mode = mode;
% sequence = '0019/'; startSeq = 620; endSeq = 630;
% sequence = '0019/'; startSeq = 925; endSeq = 937;
% sequence = '0019/'; startSeq = 944; endSeq = 939;
% sequence = '0019/'; startSeq = 609; endSeq = 620;
% sequence = '0000/'; startSeq = 56; endSeq = 70;
% sequence = '0000/'; startSeq = 1; endSeq = 10; distThd = 30;
% sequence = '0000/'; staSeq = 1; endSeq = 153; distThd = 30;
% sequence = '0000/'; startSeq = 35; endSeq = 45; distThd = 30;
% sequence = '0000/'; staSeq = 75; endSeq = 95; distThd = 30;
% sequence = '0000/'; startSeq = 70; endSeq = 79; distThd = 30;
% sequence = '0000/'; startSeq = 95; endSeq = 100; distThd = 40;
% sequence = '0000/'; startSeq = 106; endSeq = 118; distThd = 40;
% sequence = '0020/'; startSeq = 161; endSeq = 173;
% sequence = '0019/'; startSeq = 32; endSeq = 38;
% sequence = '0000/'; startSeq = 1; endSeq = 79; distThd = 30;
% sequence = '0009/'; startSeq = 240; endSeq = 255; distThd = 100;
%  sequence = '0009/'; startSeq = 255; endSeq = 265; distThd = 100;
% sequence = '0009/'; startSeq = 265; endSeq = 285; distThd = 100;
% sequence = '0009/'; startSeq = 240; endSeq = 265; distThd = 100;
% sequence = '0009/'; startSeq = 471; endSeq = 480; distThd = 100;
%  sequence = '0009/'; startSeq = 490; endSeq = 510; distThd = 100;
%  sequence = '0009/'; startSeq = 510; endSeq = 530; distThd = 100;
% sequence = '0009/'; startSeq = 530; endSeq = 550; distThd = 100;
%  sequence = '0009/'; startSeq = 550; endSeq = 570; distThd = 100;
%  sequence = '0009/'; startSeq = 650; endSeq = 670; distThd = 100;
% sequence = '0019/'; staSeq = 545; endSeq = 560; distThd = 100;
% sequence = '0019/'; startSeq = 565; endSeq = 577; distThd = 100;
%  sequence = '0010/'; staSeq = 75; endSeq = 84; distThd = 100;
% sequence = '0011/'; staSeq =160; endSeq = 170; distThd = 100;
% sequence = '0002/'; staSeq =119; endSeq = 129; distThd = 100;
sequence = '0020/'; staSeq = 102; endSeq = 110; distThd = 30;
distThd = 30; midSeq = staSeq+floor(0.5*(endSeq-staSeq));
paramDF.saveTrkName = ['DFT_', sequence(1:end-1),'_', num2str(staSeq),'_', num2str(endSeq),'.mat'];
paramDF.sequence = sequence;
paramDF.staSeq = staSeq;
paramDF.endSeq   = endSeq;
paramDF.midSeq   = midSeq;
paramDF.seqLen   = endSeq-staSeq+1;
paramDF.nFrm     = min(5,floor((endSeq-staSeq)/2));
paramDF.nExtnd   = (endSeq-staSeq+1) - paramDF.nFrm;
paramDF.distThd  = distThd;
paramDF.prjErrThd= 2;
paramDF.gndHight = -0.50; 
paramDF.incmplTrajLnth = 5;
paramDF.matches  = [];
paramDF.matchName= [sequence(1:end-1),'_', num2str(staSeq),'_', num2str(endSeq),'.txt'];

paramDF.leftFileDir = [paramDF.leftImgDir, sequence]; 
paramDF.imgsLeft    = dir([paramDF.leftFileDir,'*.png']);
paramDF.rightFileDir= [paramDF.rightImgDir, sequence]; 
paramDF.imgsRight   = dir([paramDF.rightFileDir,'*.png']);


%% Flow interpolation
paramDF.rad3d = 0.1; % flow knn radius in 3d
paramDF.rad2d = 3; % flow knn radius in 2d
paramDF.wSz   = 7; % flow visual memory size