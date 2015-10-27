%% denserFlow = flowInterpolation(flow2d, flow3d, rad2d, rad3d)
function denserFlow = flowInterpolation(flow2d, flow3d, rad2d, rad3d)

[rnnIdxTab3d, rnnDistTab3d] = rangesearch(flow3d, flow3d, rad3d);
[rnnIdxTab2d, rnnDistTab2d] = rangesearch(flow2d, flow2d, rad2d);

end