%% functions/detectClosestsWithEllipses.m
function detectClosestsWithEllipses(objectFolder, lutFolder,numPos,numCam,numImg)
% detectClosestsWithEllipses   Detect sphere centers in image coordinates per camera/position
%   detectClosestsWithEllipses(objectFolder, lutFolder,numPos,numCam,numImg)
%     objectFolder : base folder path for object images 
%     lutFolder    : base folder path for LUTs      
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)
%     numImg       : number of frames per measurement or position

    radius  = 10 * (46/(2*pi));  % mm
    X0_Y0_Z0_calc_img_cam = zeros(numCam*numPos, 12);

    for cam_no = 1:numCam
        % load LUT for this camera
        lutFile = fullfile(lutFolder, sprintf('cam%d', cam_no), 'LookUpTableImagev1.mat');
        data    = load(lutFile, 'I_LUT');
        I_LUT   = data.I_LUT;

        for pos_no = 1:numPos
            I = zeros(480,640,numImg);
            for sphereNo = 1:4
                for img_no = 1:numImg
                    if img_no==1
                        % load sphere corner coords
                        tmp = load(fullfile(objectFolder, sprintf('cam%d', cam_no), 'lrgbw.mat'), 'lrgbw');
                        lrgbw = tmp.lrgbw{pos_no};
                        lr = lrgbw(1,:);  lg = lrgbw(2,:);
                        lb = lrgbw(3,:);  lw = lrgbw(4,:);
                    end
                    % read all depth frames, replace zeros with 8191 (cameras max value)
                    fname = sprintf('depthImg%dpos%d.png', img_no, pos_no);
                    frame = imread(fullfile(objectFolder, sprintf('cam%d', cam_no), fname));
                    frame(frame==0) = 8191;
                    I(:,:,img_no) = frame;
                
                    % process each sphere color region
                        switch sphereNo
                            case 1, coords = lr;
                            case 2, coords = lg;
                            case 3, coords = lb;
                            case 4, coords = lw;
                        end
        
                        % median‐filtered and smoothed image for min detection
                        eps = 0;
                        filt = medfilt2(conv2(I(:,:,img_no), ones(7,7)/49, 'same'), [3 3], 'symmetric');
                        
                        region = I(coords(1):coords(2), coords(3):coords(4), img_no);
                        if img_no==1 [I_r, der] = remove_bottomRows(region);else; I_r=region(1:size(I_r,1),:); end
                        if size(I_r,2) ~= size(der,2), der = der(:,1:end-1); end
                        I_r(end-5:end,:) = I_r(end-5:end,:) .* der;
        
                        F = filt(coords(1):coords(2), coords(3):coords(4));
                        F = F(1:size(I_r,1), :);
                        [r,c] = size(F);
                        tol_r = round(r*0.1); tol_c = round(c*0.1);
                        Fsub = F(tol_r:end-tol_r, tol_c:end-tol_c);
                        minVal = min(Fsub(:));
                        cutVals_all{img_no,sphereNo} = I_r(I_r>=minVal & I_r<(minVal+radius+eps));
                        I_r_all{img_no,sphereNo}=I_r;
                end
            end
            for sphereNo = 1:4
                % find center using provided helper
                cutVals=cell(img_no,1);
                I_r2=cell(img_no,1);
                [cutVals{1:end}]=cutVals_all{:,sphereNo};
                [I_r2{1:end}]=I_r_all{:,sphereNo};
                
                [Xc, Yc, d1, d2, ~] = FindCenterOfClosest(cutVals, I_r2, 20);
                Zc = max(d1, d2);
                switch sphereNo
                    case 1, coords = lr;
                    case 2, coords = lg;
                    case 3, coords = lb;
                    case 4, coords = lw;
                end
                X_img(sphereNo) = (coords(3)-1) + Xc;
                Y_img(sphereNo) = (coords(1)-1) + Yc;
                Z0_calc_depthErrorRemoved=Zc;
                Z_img(sphereNo) = Z0_calc_depthErrorRemoved + radius;
            end
            idx = (pos_no-1)*numCam + cam_no;
            X0_Y0_Z0_calc_img_cam(idx,:) = [X_img, Y_img, Z_img];
        end
        clear I_LUT
        % save per-camera results
        save(fullfile(objectFolder, sprintf('cam%d', cam_no), 'closest_xpx_ypx_and_center_Z_NoDepthErrorRemvd.mat'), ...
             'X0_Y0_Z0_calc_img_cam'); 
    end
end
