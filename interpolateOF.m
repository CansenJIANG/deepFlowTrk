%% Compute OF for 2D features
% [trkOF] = interpolateOF(trkFeat, flo)
function [trkOF] = interpolateOF(trkFeat, flo)
if(size(trkFeat,1)~=length(trkFeat))
    trkFeat = trkFeat';
end
vx = flo(:,:,1); vy = flo(:,:,2); 
trkOF = zeros(size(trkFeat));
for i = 1:length(trkFeat)
    px = floor(trkFeat(i,1));
    py = floor(trkFeat(i,2));
    if(trkFeat(i,1)==px && trkFeat(i,2)==py)
%         trkFeat(i,1) = trkFeat(i,1)+vx(py, px);
%         trkFeat(i,2) = trkFeat(i,2)+vy(py, px);
        trkOF(i,1)  = vx(py, px);
        trkOF(i,2)  = vy(py, px);
    else
        if(px<1 || px>size(flo,2)-1 ||py<1 || py>size(flo,1)-1)
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
%         trkFeat(i,1) = trkFeat(i,1) + deltaX;
%         trkFeat(i,2) = trkFeat(i,2) + deltaY;
        trkOF(i,1)  = deltaX;
        trkOF(i,2)  = deltaY;
    end
end

end