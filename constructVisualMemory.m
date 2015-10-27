function paramDF = constructVisualMemory(paramDF, flow2d, flow3d)
if size(flow2d, 1) ~= length(flow2d)
    flow2d = flow2d';
end
if size(flow3d, 1) ~= length(flow3d)
    flow3d = flow3d';
end
[paramDF.rnnIdxTab3d, paramDF.rnnDistTab3d] = rangesearch(flow3d, flow3d, paramDF.rad3d);
[paramDF.rnnIdxTab2d, paramDF.rnnDistTab2d] = rangesearch(flow2d, flow2d, paramDF.rad2d);
end