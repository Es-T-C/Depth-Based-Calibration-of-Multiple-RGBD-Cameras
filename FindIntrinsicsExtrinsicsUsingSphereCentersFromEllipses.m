%% functions/FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses.m
function FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses(objectFolder,use_coordsAfterSphereFit,numPos,numCam)
% FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses  Calculate the calibration params (initials for BA)
%   FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses(objectFolder,use_coordsAfterSphereFit,numPos,numCam)
%     objectFolder : base folder path containing 'camN' subdirectories
%     use_coordsAfterSphereFit :  0: ellipse; 1: sphere fit
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)

    % Configuration
    pixelsize               = 0.0104; % mm/px
    radius                  = 10*(46/(2*pi));

    for cam_no = 1:numCam
        % Load sphere‐center coordinates
        switch use_coordsAfterSphereFit
            case 0
                X0Y0Z = load(fullfile(objectFolder, sprintf('cam%d',cam_no), ...
                                      'closest_xpx_ypx_and_center_Z_NoDepthErrorRemvd.mat'),'X0_Y0_Z0_calc_img_cam').X0_Y0_Z0_calc_img_cam;
            case 1
                X0Y0Z = load(fullfile(objectFolder, sprintf('cam%d',cam_no), ...
                                      'center_xpx_ypx_Z.mat'),'X0_Y0_Z0').X0_Y0_Z0;
        end

        % Build world coords of sphere centers
        Wr = [ 373.2122  0         0       ];
        Wg = [-186.6061  0        -323.2113];
        Wb = [-186.6061  0         323.2113];
        Ww = [ 0         527.8018  0       ];

        % Assemble per-position
        for pos_no = 1:numPos
            idx = (pos_no-1)*4 + cam_no;
            % image coords
            IMr = X0Y0Z(idx, [1 5]);
            IMg = X0Y0Z(idx, [2 6]);
            IMb = X0Y0Z(idx, [3 7]);
            IMw = X0Y0Z(idx, [4 8]);
            Im_rc(:,:,pos_no)=[IMr;IMg;IMb;IMw];

            % depths (mm)
            Zs = X0Y0Z(idx,9:12)';
            Z_RGBW_center(:,:,pos_no)=Zs;

            % last row of projection matrix (scaled)
            M_A = [Wr 1; Wg 1; Wb 1; Ww 1];
            m_last(:,pos_no) = ((M_A'*M_A)\(M_A'*Zs))/pixelsize;

            % build linear system for remaining m(1:8)
            A = [ Wr 1 zeros(1,4);
                  zeros(1,4) Wr 1;
                  Wg 1 zeros(1,4);
                  zeros(1,4) Wg 1;
                  Wb 1 zeros(1,4);
                  zeros(1,4) Wb 1;
                  Ww 1 zeros(1,4);
                  zeros(1,4) Ww 1 ];
            A_allPos(:,:,pos_no)=A;
            b = [ IMr(1)*pixelsize*[Wr 1]*m_last(:,pos_no);
                  IMr(2)*pixelsize*[Wr 1]*m_last(:,pos_no);
                  IMg(1)*pixelsize*[Wg 1]*m_last(:,pos_no);
                  IMg(2)*pixelsize*[Wg 1]*m_last(:,pos_no);
                  IMb(1)*pixelsize*[Wb 1]*m_last(:,pos_no);
                  IMb(2)*pixelsize*[Wb 1]*m_last(:,pos_no);
                  IMw(1)*pixelsize*[Ww 1]*m_last(:,pos_no);
                  IMw(2)*pixelsize*[Ww 1]*m_last(:,pos_no) ];

            m12 = (A'*A)\(A'*b);
            m    = [m12; m_last(:,pos_no)];
            M    = [m(1:4)'; m(5:8)'; m(9:12)'];

            % normalize so that third row is unit
            sigma = norm(M(3,1:3));
            M = M / sigma;

            % decompose for intrinsics
            q1 = M(1,1:3)'; q2 = M(2,1:3)'; q3 = M(3,1:3)';
            ox(pos_no) = dot(q1,q3); 
            oy(pos_no) = dot(q2,q3);
            fx(pos_no) = sqrt(dot(q1,q1) - ox(pos_no)^2);
            fy(pos_no) = sqrt(dot(q2,q2) - oy(pos_no)^2);
            fx_px = fx / pixelsize; fy_px = fy / pixelsize;
            ox_px = ox / pixelsize; oy_px = oy / pixelsize;

            % extrinsics
            R_new(:,:,pos_no) = [ ...
                (ox(pos_no)*M(3,1)-M(1,1))/fx(pos_no), (ox(pos_no)*M(3,2)-M(1,2))/fx(pos_no), (ox(pos_no)*M(3,3)-M(1,3))/fx(pos_no); 
                (oy(pos_no)*M(3,1)-M(2,1))/fy(pos_no), (oy(pos_no)*M(3,2)-M(2,2))/fy(pos_no), (oy(pos_no)*M(3,3)-M(2,3))/fy(pos_no); 
                M(3,1),                M(3,2),                M(3,3)                ];
            T_new(:,:,pos_no) = [ ...
                (ox(pos_no)*m(12) - m(4))/fx(pos_no);
                (oy(pos_no)*m(12) - m(8))/fy(pos_no);
                 m(12) ];

        end

        ox_avg=mean(ox);oy_avg=mean(oy);
        fx_avg=mean(fx);fy_avg=mean(fy);
        
        [R_new,T_new,~]=Pw_Pc_forCenters(Im_rc,ox_avg,oy_avg,fx_avg,fy_avg,A_allPos,Z_RGBW_center,m_last,numPos);

        % save to disk
        if use_coordsAfterSphereFit==0
            save(fullfile(objectFolder, sprintf('cam%d',cam_no), ...
                 'initialParams_FromProjectionMatrixMethod.mat'), ...
                 'fx_px','fy_px','ox_px','oy_px','pixelsize','R_new','T_new','radius');
        else
            save(fullfile(objectFolder, sprintf('cam%d',cam_no), ...
                 'initialParams_FromProjectionMatrixMethodAndSphereFitting.mat'), ...
                 'fx_px','fy_px','ox_px','oy_px','pixelsize','R_new','T_new','radius');
        end
    end
end
