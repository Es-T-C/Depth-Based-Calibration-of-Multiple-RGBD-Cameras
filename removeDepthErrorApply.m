%% functions/removeDepthErrorApply.m
function removeDepthErrorApply(objectFolder, lutFolder,numPos,numCam,numImg,imgType1,imgType2)
% removeDepthErrorApply   Apply depth error removal
%   removeDepthErrorApply(objectFolder, lutFolder,numPos,numCam,numImg,imgType1,imgType2)
%     objectFolder : base folder path with raw and sphere-only images
%     lutFolder    : base folder path containing LUT .mat files
%     numCam     : number of calibrated cameras
%     numPos     : number of positions (object)
%     numImg     : number of frames per measurement or position
%     imgType1   : image type, 1 -> raw images, 2 -> sphere-only images, 3 -> flatwall images 
%     imgType2   : image type, 1 -> median sphere-only images, 2 -> sphere-only images

    % Configuration flags
    if imgType1==1
        spheresOnlyEnable       = false;   % apply to raw images
    elseif imgType1==2
        spheresOnlyEnable       = true;   % apply to sphere-only images
        if imgType2==1
            spheresOnlyMedianEnable = true;   % apply to median sphere-only images
        elseif imgType2==2
            spheresOnlyMedianEnable = false;   % apply to sphere-only images
        end
    elseif imgType1==3
        flatwallImgEnable=true;
        numImg=25;
    end

    for cam_no = 1:numCam
        fprintf('Applying depth error removal for camera %d...\n', cam_no);
        % Load LUT volume from disk
        lutFile = fullfile(lutFolder, sprintf('cam%d', cam_no), 'LookUpTableImagev1.mat');
        data    = load(lutFile, 'I_LUT');
        I_LUT   = data.I_LUT;
        if flatwallImgEnable
            for img_no=1:numImg
                flatwallFile = fullfile(lutFolder, sprintf('cam%d', cam_no), ...
                                    sprintf('depthImg100_median_meas_%d.png', img_no));
                I_depth      = double(imread(flatwallFile));
                outImg = removeDepthError(I_depth, I_LUT);
                outName = sprintf('depthImg100_median_meas_%d_DepthErrorRemvdFlatWall.png', img_no);
                imwrite(uint16(outImg), fullfile(lutFolder, sprintf('cam%d', cam_no), outName));
            end
        else
            for pos_no = 1:numPos
                % Read sphere-only median image
                sphereMedFile = fullfile(objectFolder, sprintf('cam%d', cam_no), ...
                                         sprintf('depthImgMedian_pos%d_spheresOnly.png', pos_no));
                I_spheres     = double(imread(sphereMedFile));
        
                for img_no = 1:numImg
                    rawDepthFile = fullfile(objectFolder, sprintf('cam%d', cam_no), ...
                                            sprintf('depthImg%dpos%d.png', img_no, pos_no));
                    I_depth      = double(imread(rawDepthFile));
    
                    if spheresOnlyEnable
                        I_temp = I_spheres;
                        if ~spheresOnlyMedianEnable
                            I_temp(I_temp>0) = 1;
                            I_depth = I_temp .* I_depth;
                            outImg = removeDepthError(I_depth, I_LUT);
                            outName = sprintf('depthImg%dpos%d_spheresOnly_DepthErrorRemvdFlatWall.png', img_no, pos_no);
                        else
                            outImg = removeDepthError(I_temp, I_LUT);
                            outName = sprintf('depthImgMedian_pos%d_spheresOnly_DepthErrorRemvdFlatWall.png', pos_no);
                            imwrite(uint16(outImg), fullfile(objectFolder, sprintf('cam%d', cam_no), outName));
                            break; % only first median image
                        end
                    else
                        outImg = removeDepthError(I_depth, I_LUT);
                        outName = sprintf('depthImg%dpos%d_DepthErrorRemvdFlatWall.png', img_no, pos_no);
                    end
        
                    imwrite(uint16(outImg), fullfile(objectFolder, sprintf('cam%d', cam_no), outName));
                end
            end
        end
    end
end
