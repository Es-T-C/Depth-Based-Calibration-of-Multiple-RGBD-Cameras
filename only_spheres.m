%% functions/only_spheres.m
function only_spheres(folderPath,numPos,numCam)
% ONLY_SPHERES   Extract only sphere regions from depth images
%   only_spheres(folderPath,numPos,numCam)
%     folderPath : base folder path containing 'camN' subdirectories
%     numCam     : number of calibrated cameras
%     numPos     : number of positions (object)
    
    loadCoordsEnable  = (inputdlg('Do you want to load precalculated coordinates?(Y/N):','Removal Settings',[1 50],{'Y'}));

    % Constant
    radius     = 10 * (46/(2*pi));   % mm
    
    % Loop over cameras and positions
    for cam_no = 1:numCam
        camFolder = fullfile(folderPath, sprintf('cam%d', cam_no));
        for pos_no = 1:numPos
            % Read depth median image
            depthFile = fullfile(camFolder, sprintf('depthImgMedian_pos%d.png', pos_no));
            I_depth  = double(imread(depthFile));
            if loadCoordsEnable{1}=='Y'
                load([folderPath '\cam' int2str(cam_no) '\lrgbw.mat'])
                lr=lrgbw{pos_no}(1,:);lg=lrgbw{pos_no}(2,:);lb=lrgbw{pos_no}(3,:);lw=lrgbw{pos_no}(4,:);
            else
                fprintf('Processing camera %d, position %d...\n', cam_no, pos_no);
                close all
                % Read and display color image
                colorFile = fullfile(camFolder, sprintf('colorImg1pos%d.png', pos_no));
                figure; imshow(imread(colorFile));
    
                % Get sphere corner points (order: R G B W)
                figure;
                pts = round(readPoints(I_depth, 8));
                lr = [pts(2,1) pts(2,2) pts(1,1) pts(1,2)];
                lg = [pts(2,3) pts(2,4) pts(1,3) pts(1,4)];
                lb = [pts(2,5) pts(2,6) pts(1,5) pts(1,6)];
                lw = [pts(2,7) pts(2,8) pts(1,7) pts(1,8)];
                lrgbw{pos_no} = [lr; lg; lb; lw];
            end

            % Initialize storage for region depths
            minVals = zeros(4, 1);
            I_r_cells = cell(4,1);

            % Process each color region
            regions = {lr, lg, lb, lw};
            for i = 1:4
                coords = regions{i};
                I_region = I_depth(coords(1):coords(2), coords(3):coords(4));

                % Trim bottom rows
                [I_r, derLast] = remove_bottomRows(I_region);
                if size(I_r,2) ~= size(derLast,2)
                    derLast = derLast(:,1:end-1);
                end

                % Compute local threshold
                center = round(size(I_r)/2);
                subRegion = I_r(center(1)-15:center(1)+15, center(2)-15:center(2)+15);
                minVals(i) = min(subRegion(subRegion>0));

                % Apply derivative mask
                I_r(end-5:end, :) = I_r(end-5:end, :) .* derLast;
                I_r_cells{i} = I_r;
            end

            % Create binary mask for spheres
            I_onlySphere = zeros(size(I_depth,1), size(I_depth,2), 4);
            for i = 1:4
                I_r    = I_r_cells{i};
                coords = regions{i};
                minVal = median(minVals(i));
                mask   = I_r;
                mask(mask < minVal)         = 0;
                mask(mask > minVal + radius)= 0;
                mask(mask > 0)              = 1;

                removedRows = size(I_depth(coords(1):coords(2),:),1) - size(I_r,1);
                I_onlySphere(coords(1):coords(2)-removedRows, coords(3):coords(4), i) = mask;
            end

            % Combine and save sphere-only depth image
            I_combined = sum(I_onlySphere,3) .* I_depth;
            outFile = fullfile(camFolder, sprintf('depthImgMedian_pos%d_spheresOnly.png', pos_no));
            imwrite(uint16(I_combined), outFile);
        end

        % Save sphere corner coordinates
        save(fullfile(folderPath, sprintf('cam%d', cam_no), 'lrgbw.mat'), 'lrgbw');
    end
end
