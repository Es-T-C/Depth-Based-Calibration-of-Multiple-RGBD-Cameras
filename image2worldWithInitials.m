%% functions/image2worldWithInitials.m
function image2worldWithInitials(objectFolder,numPos,numCam)
% image2worldWithInitials   Transform depth pixels to world coordinates per camera/position
%   image2worldWithInitials(objectFolder,numPos,numCam)
%     objectFolder : base folder path containing 'camN' subdirectories
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)

    pixelsize = 0.0104;               % mm/px

    for cam_no = 1:numCam
        camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
        params    = load(fullfile(camFolder, 'initialParams_FromProjectionMatrixMethodAndSphereFitting.mat'));
        fx = mean(params.fx_px) * pixelsize;
        fy = mean(params.fy_px) * pixelsize;
        ox = mean(params.ox_px) * pixelsize;
        oy = mean(params.oy_px) * pixelsize;
        R_new = params.R_new;
        T_new = params.T_new;

        for pos_no = 1:numPos
            depthFile = fullfile(camFolder, sprintf('depthImgMedian_pos%d_spheresOnly_DepthErrorRemvdFlatWall.png', pos_no));
            I_depth   = double(imread(depthFile));
            [rows, cols] = size(I_depth);

            ptsWorld = zeros(rows*cols, 3);
            count = 0;
            for r = 1:rows
                for c = 1:cols
                    d = I_depth(r,c);
                    if d > 0
                        count = count + 1;
                        x_cam = ((c*pixelsize - ox) * d) / -fx;
                        y_cam = ((r*pixelsize - oy) * d) / -fy;
                        z_cam = d;
                        camPt = [x_cam; y_cam; z_cam];
                        worldPt = R_new(:,:,pos_no) \ (camPt - T_new(:,:,pos_no));
                        ptsWorld(count, :) = worldPt';
                    end
                end
            end
            ptsWorld = unique(ptsWorld(1:count, :), 'rows', 'stable');
            pc = pointCloud(ptsWorld);
            pc = outlierRemoval_func(pc, 30, 5);
            pc = Smoothing_3D_func(pc);
            pcwrite(pc, fullfile(camFolder, sprintf('pos%d_initialParams_FromProjectionMatrixMethod.ply', pos_no)));
        end
    end
end
