%% functions/DetectSpherePC_WithInitialParams_onSphereImages.m
function DetectSpherePC_WithInitialParams_onSphereImages(objectFolder,numCam,numPos,use_paramsAfterSphereFit)
% DETECTSPHEREPC_WITHINITIALPARAMS_ONSPHEREIMAGES   Refine sphere centers via point-cloud sphere fitting
%   DetectSpherePC_WithInitialParams_onSphereImages(objectFolder,Num_cam,Num_pos,use_paramsAfterSphereFit)
%     objectFolder : base folder path containing 'camN' subdirectories with images and initial params
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)
%     use_paramsAfterSphereFit : % true -> sphere fitting params, false -> projection matrix params

    radius                   = 10*(46/(2*pi)); % mm
    maxDistance2FitSphere    = 3;
    Num_iter=100;
    X0_Y0_Z0=zeros(32,12);
    i_row=0;
    for pos_no = 1:numPos
        for cam_no = 1:numCam
            camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
            if ~use_paramsAfterSphereFit
                S = load(fullfile(camFolder, 'initialParams_FromProjectionMatrixMethod.mat'));
            else
                S = load(fullfile(camFolder, 'initialParams_FromProjectionMatrixMethodAndSphereFitting.mat'));
            end
            fx_px = S.fx_px; fy_px = S.fy_px;
            ox_px = S.ox_px; oy_px = S.oy_px;

            tmp      = load(fullfile(camFolder, 'lrgbw.mat'),'lrgbw');
            lrgbwRow = tmp.lrgbw{pos_no};
            lr = lrgbwRow(1,:); lg = lrgbwRow(2,:);
            lb = lrgbwRow(3,:); lw = lrgbwRow(4,:);

            depthFile = fullfile(camFolder, sprintf('depthImgMedian_pos%d_spheresOnly_DepthErrorRemvdFlatWall.png', pos_no));
            I_depth   = double(imread(depthFile));
            regions    = {lr, lg, lb, lw};
            sphereParams = zeros(Num_iter,4,4);
            for i = 1:4
                coords = regions{i};
                region = I_depth(coords(1):coords(2), coords(3):coords(4));
                region_tmp=region;
                minVal = min(min(region_tmp(region_tmp>0)));
                region_tmp(region_tmp < minVal)           = 0;
                region_tmp(region_tmp > minVal + radius)  = 0;
                region=region_tmp;
                I_region=zeros(480,640);
                I_region(coords(1):coords(2), coords(3):coords(4))=region;
                [rows, cols] = find(I_region>0);
                depths       = I_region(I_region>0);
                pts = [(cols - mean(ox_px)) .* depths / -mean(fx_px), ...
                       (rows - mean(oy_px)) .* depths / -mean(fy_px), ...
                       depths];
                pc  = pointCloud(pts);
                pc_clean = outlierRemoval_func(pc, 30, 5);
                pc_smooth = Smoothing_3D_func(pc_clean);

                for iter = 1:Num_iter
                    model = pcfitsphere(pc_smooth, maxDistance2FitSphere);
                    sphereParams(iter,:,i) = model.Parameters;
                end
            end
            x=zeros(1,4);y=zeros(1,4);z=zeros(1,4);
            for i = 1:4
                diffs = abs(sphereParams(:,:,i) - radius);
                [~, idx] = min(diffs(:,4));
                model = sphereModel(sphereParams(idx,:,i));
                x(i)=-(model.Center(1)*mean(fx_px)/model.Center(3))+mean(ox_px);
                y(i)=-(model.Center(2)*mean(fy_px)/model.Center(3))+mean(oy_px);
                z(i)=model.Center(3);
            end
            i_row=i_row+1;
            X0_Y0_Z0(i_row,:) = [x y z];
        end
    end
    for cam_no=1:numCam
        camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
        save(fullfile(camFolder, 'center_xpx_ypx_Z.mat'), 'X0_Y0_Z0');
    end
end
