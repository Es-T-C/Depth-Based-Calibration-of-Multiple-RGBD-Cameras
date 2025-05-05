%% functions/image2worldWithAllParams.m
function image2worldWithAllParams(objectFolder,numPos,numCam)
% image2worldWithAllParams...   Transform depth pixels to world coordinates per camera/position
%   image2worldWithAllParams(objectFolder,numPos,numCam)
%     objectFolder : base folder path containing 'camN' subdirectories
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)

    for method_no=0:1:3 % 0 -> for initials, 1 -> for BAin2D, 2->for BAin3D_M1, 3->for BAin3D_M2
        if method_no==0
            p=load('params_initial.mat','par').par;load("rot_matrices_BAin2D.mat","R1","R2","R3","R4");
            pt_title='initial';
        elseif method_no==1
            load('params_BA_in2D.mat','p');load("rot_matrices_BAin2D.mat","R1","R2","R3","R4");
            pt_title='BA_in2D';
        elseif method_no==2
            load('params_BA_in3D_M1.mat','p');load("rot_matrices_BAin3D_M1.mat","R1","R2","R3","R4");
            pt_title='BA_in3D_M1';
        else
            load('params_BA_in3D_M2.mat','p');load("rot_matrices_BAin3D_M2.mat","R1","R2","R3","R4");
            pt_title='BA_in3D_M2';
        end
        disp(['   -> Point clouds are creating -> ' pt_title])

        R_tmp=cell(1,4);
        [R_tmp{:}]=deal([R1], [R2], [R3], [R4]);
        pixel_size=0.0104;%mm/px
    
        for pos_no=1:1:numPos
            for cam_no=1:1:numCam
                %% parameters
                par_cam=p((cam_no-1)*(6+numPos*6)+1:cam_no*(6+numPos*6));
                fx_mm=par_cam(1);fy_mm=par_cam(2);ox_mm=par_cam(3);oy_mm=par_cam(4);k1=par_cam(5);k2=par_cam(6);
                wx=par_cam((pos_no-1)*6+6+1);wy=par_cam((pos_no-1)*6+6+2);wz=par_cam((pos_no-1)*6+6+3);
                tx=par_cam((pos_no-1)*6+6+4);ty=par_cam((pos_no-1)*6+6+5);tz=par_cam((pos_no-1)*6+6+6);
                fx_px=fx_mm/pixel_size;fy_px=fy_mm/pixel_size;ox_px=ox_mm/pixel_size;oy_px=oy_mm/pixel_size;
                theta=sqrt(wx^2+wy^2+wz^2);
                w=[wx wy wz]/theta;
                Q=[cos(theta)+w(1)*w(1)*(1-cos(theta))     ,  w(1)*w(2)*(1-cos(theta))-w(3)*sin(theta),  w(1)*w(3)*(1-cos(theta))+w(2)*sin(theta);
                       w(2)*w(1)*(1-cos(theta))+w(3)*sin(theta),  cos(theta)+w(2)*w(2)*(1-cos(theta))     ,  w(2)*w(3)*(1-cos(theta))-w(1)*sin(theta);
                       w(3)*w(1)*(1-cos(theta))-w(2)*sin(theta),  w(3)*w(2)*(1-cos(theta))+w(1)*sin(theta),  cos(theta)+w(3)*w(3)*(1-cos(theta))];
                R_all=R_tmp{cam_no};
                R=Q*R_all(:,:,pos_no);
                T=[tx;ty;tz];
                camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
                I_cloud=double(imread([camFolder '\depthImgMedian_pos' int2str(pos_no) '_spheresOnly_DepthErrorRemvdFlatWall.png']));
                [row ,col]=size(I_cloud);
    
                I_cloud=rect_img([fx_px fy_px],[ox_px oy_px],[k1; k2],I_cloud);
            
                i=0;
                xyzPoints=[];xyzPoints_world=[];
                for m=1:1:row
                    for l=1:1:col
                        if I_cloud(m,l)==0  
                            continue;
                        else
                            xyzPoints(1,1)=((l*pixel_size-(ox_mm))*I_cloud(m,l))/-(fx_mm);
                            xyzPoints(1,2)=((m*pixel_size-(oy_mm))*I_cloud(m,l))/-(fy_mm);
                            xyzPoints(1,3)= I_cloud(m,l);
                            temp=R\(xyzPoints'-T); %for world coordinates
                            if temp(2)>-75
                                i=i+1;
                                xyzPoints_world(i,:)=temp(1:3)';
                            end
                        end
                    end
                end
                [xyzPoints_world,~,~] = unique(xyzPoints_world,'rows','stable');
                ptCloud = pointCloud(xyzPoints_world(:,1:3));
    
                ptCloud=outlierRemoval_func(ptCloud,30,5);
                ptCloud=Smoothing_3D_func(ptCloud);
                ptCloud_all{cam_no}=ptCloud;
    
                camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
                pcwrite(ptCloud,[camFolder '\pos' int2str(pos_no) '_Params_' pt_title '.ply']);
            end
        end
    end
end