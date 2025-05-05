%% functions/FlatWall2LUTs.m
function FlatWall2LUTs(folderPath,numCam,numMeas)
% FlatWall2LUTs   Compute lookup tables (LUTs) for depth error removal
%   FlatWall2LUTs(folderPath,numCam,numMeas)
%     folderPath : base folder path containing 'camN' subdirectories
%     numCam     : number of calibrated cameras
%     numMeas    : number of frames per measurement or position

    for cam_no = 1:numCam
        fprintf('Processing camera %d LUT...\\n', cam_no);
        % Initialize depth‐means and LUT volume
        I_depths = zeros(480, 640, numMeas, 'double');
        I_LUT    = zeros(480, 640, 2060, 'double');
        camFolder = fullfile(folderPath, sprintf('cam%d', cam_no));

        % Build local mean depth per measurement
        for meas_no = 1:numMeas
            outName = sprintf('depthImg100_median_meas_%d.png', meas_no);
            I = imread(fullfile(camFolder, outName));
            for r = 2:478
                for c = 6:635
                    I_depths(r,c,meas_no) = mean(mean(I(r-1:r+1, c-1:c+1)));
                end
            end
        end

        % Load true depth levels for this camera
        depthFile = fullfile(folderPath, sprintf('cam%d', cam_no), 'depth_levels.txt');
        fid = fopen(depthFile, 'r');
        d   = fscanf(fid, '%f');
        fclose(fid);
        depths = [750; d; 2050];

        % Temporary array for interpolation
        tmp = zeros(1, numMeas+2);

        % Populate LUT by linear interpolation
        for c = 6:635
            for r = 2:478
                for pd=800:1:2060
                    %% find coords. and closest pixel coord.
                    tmp(2:end-1)=I_depths(r,c,:);
                    tmp(1)=tmp(2)-mean(tmp(3:end-1)-tmp(2:end-2));
                    tmp(end)=tmp(end-1)+mean(tmp(3:end-1)-tmp(2:end-2));
                            
                    [idx1]=find(abs(pd-tmp)==min(abs(pd-tmp)));
                    idx1_length=length(idx1);
        
                    if (idx1==1) & (pd<tmp(idx1))
                        continue;
                    elseif (idx1==27) & (pd>tmp(idx1))
                        continue;
                    elseif idx1_length>1
                        idx1_tmp=idx1;
                        idx1=idx1_tmp(1);
                        idx2=idx1_tmp(end);
                    else
                        if pd>tmp(idx1)
                            idx2=idx1+1;
                        elseif pd<tmp(idx1)
                            idx2=idx1-1;
                        else
                            [idx2]=idx1;
                        end
                    end
                    %% distances for interpolation
                    dist1=abs(pd-tmp(idx1));
                    dist2=abs(pd-tmp(idx2));
                    dist_full=abs(tmp(idx2)-tmp(idx1));
                    %% depth interpolation
                    if idx2==idx1
                        I_LUT(r,c,pd)=depths(idx1);
                    else
                        I_LUT(r,c,pd)=(depths(idx1)*dist2+depths(idx2)*dist1)/dist_full;
                    end
                end
            end
        end
        % Save LUT volume to file
        save(fullfile(folderPath, sprintf('cam%d', cam_no), 'LookUpTableImagev1.mat'), 'I_LUT', '-v7.3');
    end
end
