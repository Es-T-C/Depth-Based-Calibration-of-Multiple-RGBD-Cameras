%% functions/world2image_sphere_sampling_new.m
function world2imageSpherePointSampling(objectFolder,numPos,numCam)
% world2imageSpherePointSampling   Sample cross‐shape points on sphere surfaces
%   world2imageSpherePointSampling(objectFolder,numPos,numCam)
%     objectFolder : base folder path containing 'camN' subdirectories
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)
    
    pixelsize = 0.0104;               
    radius    = 10*(46/(2*pi));       % mm
    Num_pixel = 8;
    Num_sphere= 4;
    total_px  = (Num_pixel/2*4+1)*Num_sphere;

    x0_y0_z0_img    =zeros(numCam*numPos*(total_px),3);
    x0_y0_z0_real   =zeros(numCam*numPos*(total_px),3);
    XYZcam_all      =zeros(numCam*numPos*3, total_px);
    XYZworld_all    =zeros(numCam*numPos*3, total_px);
    
    for cam_no=1:1:numCam
        for pos_no=1:1:numPos
            camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
            load(fullfile(camFolder, 'initialParams_FromProjectionMatrixMethodAndSphereFitting.mat'),'fx_px','fy_px','ox_px','oy_px','R_new','T_new');
            %% parameters
            k1=0;k2=0;
            fx_mm=mean(fx_px)*pixelsize;fy_mm=mean(fy_px)*pixelsize;ox_mm=mean(ox_px)*pixelsize;oy_mm=mean(oy_px)*pixelsize;
            
            R = R_new(:,:,pos_no);
            T = T_new(:,:,pos_no);
            
            %% world coordinates and radius of spheres rgbw
            a_R=373.2122; b_R=0; c_R=0;
            a_G=-186.6061; b_G=0; c_G=-323.2113;
            a_B=-186.606; b_B=0; c_B=323.2113;
            a_W=0; b_W=527.8018; c_W=0;
            
            a=[a_R a_G a_B a_W];
            b=[b_R b_G b_B b_W];
            c=[c_R c_G c_B c_W];
            
            %% create spheres on world coord.
            [X,Y,Z] = sphere(500);
            Xw_R=X*radius+a_R;Yw_R=Y*radius+b_R;Zw_R=Z*radius+c_R;
            Xw_G=X*radius+a_G;Yw_G=Y*radius+b_G;Zw_G=Z*radius+c_G;
            Xw_B=X*radius+a_B;Yw_B=Y*radius+b_B;Zw_B=Z*radius+c_B;
            Xw_W=X*radius+a_W;Yw_W=Y*radius+b_W;Zw_W=Z*radius+c_W;
            Xw_all=[Xw_R;Xw_G;Xw_B;Xw_W];
            Yw_all=[Yw_R;Yw_G;Yw_B;Yw_W];
            Zw_all=[Zw_R;Zw_G;Zw_B;Zw_W];
            
            %% transform (spheres and centers) world to camera and image coordinates
            XYZ_cam=zeros(3,size(Xw_all,1)*size(Xw_all,2));x=zeros(1,size(Xw_all,1)*size(Xw_all,2));y=x;iter=0;
            for i=1:1:size(Xw_all,1)
                for j=1:1:size(Xw_all,2)
                    iter=iter+1;
                    XYZ_cam(:,iter)=[R T]*[Xw_all(i,j); Yw_all(i,j); Zw_all(i,j); 1];
                    X_prime=XYZ_cam(1,iter)/XYZ_cam(3,iter);
                    Y_prime=XYZ_cam(2,iter)/XYZ_cam(3,iter);
                    r=sqrt(X_prime^2+Y_prime^2);
                    X_prime_dist=X_prime*(1+k1*r^2+k2*r^4);
                    Y_prime_dist=Y_prime*(1+k1*r^2+k2*r^4);
                    x(iter)=-fx_mm*X_prime_dist+ox_mm;
                    y(iter)=-fy_mm*Y_prime_dist+oy_mm;
                end
            end
            xy=[x;y];
            XYZ_cam_cntr=zeros(3,4);x_cntr=zeros(1,4);y_cntr=x_cntr;iter=0;
            for i=1:1:4
                iter=iter+1;
                XYZ_cam_cntr(:,iter)=[R T]*[a(i); b(i); c(i); 1];
                X_prime=XYZ_cam_cntr(1,iter)/XYZ_cam_cntr(3,iter);
                Y_prime=XYZ_cam_cntr(2,iter)/XYZ_cam_cntr(3,iter);
                r=sqrt(X_prime^2+Y_prime^2);
                X_prime_dist=X_prime*(1+k1*r^2+k2*r^4);
                Y_prime_dist=Y_prime*(1+k1*r^2+k2*r^4);
                x_cntr(iter)=-fx_mm*X_prime_dist+ox_mm;
                y_cntr(iter)=-fy_mm*Y_prime_dist+oy_mm;
            end
            xy_img_cntr=[x_cntr;y_cntr];
            
            %% mm to pixel
            xy_img_cntr=xy_img_cntr/pixelsize;
            xy=(xy/pixelsize);
            
            %% cut unnecessary pixels
            I_new=(zeros(480,640));
            for i=1:1:size(xy,2)
                if i>0 && i<=size(xy,2)/4
                    cut_lim=XYZ_cam_cntr(1,1)^2+XYZ_cam_cntr(2,1)^2+XYZ_cam_cntr(3,1)^2;%XYZ_cam_cntr(3,1);%
                elseif i>size(xy,2)/4 && i<=2*(size(xy,2)/4)
                    cut_lim=XYZ_cam_cntr(1,2)^2+XYZ_cam_cntr(2,2)^2+XYZ_cam_cntr(3,2)^2;%XYZ_cam_cntr(3,2);%
                elseif i>2*(size(xy,2)/4) && i<=3*(size(xy,2)/4)
                    cut_lim=XYZ_cam_cntr(1,3)^2+XYZ_cam_cntr(2,3)^2+XYZ_cam_cntr(3,3)^2;%XYZ_cam_cntr(3,3);%
                elseif i>3*(size(xy,2)/4) && i<=4*(size(xy,2)/4)
                    cut_lim=XYZ_cam_cntr(1,4)^2+XYZ_cam_cntr(2,4)^2+XYZ_cam_cntr(3,4)^2;%XYZ_cam_cntr(3,4);%
                end
                if XYZ_cam(1,i)^2+XYZ_cam(2,i)^2+XYZ_cam(3,i)^2<cut_lim && round(xy(2,i))>0 && round(xy(1,i))>0 %XYZ_cam(3,i)<cut_lim
                    if round(xy(2,i))<1 || round(xy(2,i))>480 || round(xy(1,i))<1 || round(xy(1,i))>640 
                        continue;
                    else
                        I_new(round(xy(2,i)),round(xy(1,i)))=XYZ_cam(3,i);
                    end
                end
            end
            I_new_bw=I_new;
            I_new_bw(I_new_bw>0)=1;
            conn=bwconncomp(I_new_bw);
            for i=1:1:conn.NumObjects
                idxx=conn.PixelIdxList{i}(:);
                [y_tmp,x_tmp] = ind2sub(size(I_new_bw),idxx);
                [~, id]=min(sqrt((mean(x_tmp)-xy_img_cntr(1,:)).^2+(mean(y_tmp)-xy_img_cntr(2,:)).^2));
                if id==1
                    idx_r=idxx;
                elseif id==2
                    idx_g=idxx;
                elseif id==3
                    idx_b=idxx;
                elseif id==4
                    idx_w=idxx;
                end
            end
            [y_tmp_r,x_tmp_r] = ind2sub(size(I_new),idx_r);
            [y_tmp_g,x_tmp_g] = ind2sub(size(I_new),idx_g);
            [y_tmp_b,x_tmp_b] = ind2sub(size(I_new),idx_b);
            [y_tmp_w,x_tmp_w] = ind2sub(size(I_new),idx_w);
            
            x_r=round(min(x_tmp_r)+1:(max(x_tmp_r)-1-(min(x_tmp_r)+1))/Num_pixel:max(x_tmp_r)-1);
            y_r=round(min(y_tmp_r)+1:(max(y_tmp_r)-1-(min(y_tmp_r)+1))/Num_pixel:max(y_tmp_r)-1);
            x_g=round(min(x_tmp_g)+1:(max(x_tmp_g)-1-(min(x_tmp_g)+1))/Num_pixel:max(x_tmp_g)-1);
            y_g=round(min(y_tmp_g)+1:(max(y_tmp_g)-1-(min(y_tmp_g)+1))/Num_pixel:max(y_tmp_g)-1);
            x_b=round(min(x_tmp_b)+1:(max(x_tmp_b)-1-(min(x_tmp_b)+1))/Num_pixel:max(x_tmp_b)-1);
            y_b=round(min(y_tmp_b)+1:(max(y_tmp_b)-1-(min(y_tmp_b)+1))/Num_pixel:max(y_tmp_b)-1);
            x_w=round(min(x_tmp_w)+1:(max(x_tmp_w)-1-(min(x_tmp_w)+1))/Num_pixel:max(x_tmp_w)-1);
            y_w=round(min(y_tmp_w)+1:(max(y_tmp_w)-1-(min(y_tmp_w)+1))/Num_pixel:max(y_tmp_w)-1);
            
            %% cross pixels
            I_new2=zeros(480,640);
            I_new2(y_r,x_r(Num_pixel/2+1))=I_new(y_r,x_r(Num_pixel/2+1));
            I_new2(y_r(Num_pixel/2+1),x_r)=I_new(y_r(Num_pixel/2+1),x_r);
            I_new2(y_g,x_g(Num_pixel/2+1))=I_new(y_g,x_g(Num_pixel/2+1));
            I_new2(y_g(Num_pixel/2+1),x_g)=I_new(y_g(Num_pixel/2+1),x_g);
            I_new2(y_b,x_b(Num_pixel/2+1))=I_new(y_b,x_b(Num_pixel/2+1));
            I_new2(y_b(Num_pixel/2+1),x_b)=I_new(y_b(Num_pixel/2+1),x_b);
            I_new2(y_w,x_w(Num_pixel/2+1))=I_new(y_w,x_w(Num_pixel/2+1));
            I_new2(y_w(Num_pixel/2+1),x_w)=I_new(y_w(Num_pixel/2+1),x_w);
            
            %%     
            idx=find(I_new2>0);
            [y,x] = ind2sub(size(I_new2),idx);
            XYZworld=zeros(3,length(idx));
        
            for i=1:1:length(idx)
                z=I_new2(idx(i));
                Xc=(x(i)-mean(ox_px))*z/-mean(fx_px);
                Yc=(y(i)-mean(oy_px))*z/-mean(fy_px);
                Zc=z;
                XYZcamera=[Xc;Yc;Zc];
                XYZworld(:,i)=R\(XYZcamera-T);
            end
            %%     
            pc=pcread(fullfile(camFolder, sprintf('pos%d_initialParams_FromProjectionMatrixMethod.ply', pos_no)));
            pc=Interpolation3D(pc,0,10);
            xyz=round(pc.Location);
            xyzcam=zeros(3,length(xyz));I_depth=zeros(480,640);
            x_new1=zeros(length(xyz),1);y_new1=zeros(length(xyz),1);z_new1=zeros(length(xyz),1);
            for i=1:1:length(xyz)
                xyzcam(:,i)=[R T]*[xyz(i,:)'; 1];
                x_prime=xyzcam(1,i)/xyzcam(3,i);
                y_prime=xyzcam(2,i)/xyzcam(3,i);
                x_new1(i,1)=round(-mean(fx_px)*x_prime+mean(ox_px));
                y_new1(i,1)=round(-mean(fy_px)*y_prime+mean(oy_px));
                z_new1(i,1)=xyzcam(3,i);
                I_depth(y_new1(i,1),x_new1(i,1))=z_new1(i,1);
            end
            %%     
            XYZcam=zeros(3,length(idx));
            x_new=zeros(length(idx),1);y_new=zeros(length(idx),1);z_new=zeros(length(idx),1);z_img=zeros(length(idx),1);
            for i=1:1:length(idx)
                XYZcam(:,i)=[R T]*[XYZworld(:,i); 1];
                X_prime=XYZcam(1,i)/XYZcam(3,i);
                Y_prime=XYZcam(2,i)/XYZcam(3,i);
                x_new(i,1)=round(-mean(fx_px)*X_prime+mean(ox_px));
                y_new(i,1)=round(-mean(fy_px)*Y_prime+mean(oy_px));
                z_new(i,1)=XYZcam(3,i);
                z_img(i,1)=I_depth(y_new(i,1),x_new(i,1));
            end
            
            XYZcam_all((pos_no-1)*3*4+(cam_no-1)*3+1:(pos_no-1)*3*4+(cam_no)*3,:)=XYZcam;
            XYZworld_all((pos_no-1)*3*4+(cam_no-1)*3+1:(pos_no-1)*3*4+(cam_no)*3,:)=XYZworld;
            
            x0_y0_z0_img((pos_no-1)*total_px*4+(cam_no-1)*total_px+1:(pos_no-1)*total_px*4+(cam_no)*total_px,:)=[x_new y_new z_img];
            x0_y0_z0_real((pos_no-1)*total_px*4+(cam_no-1)*total_px+1:(pos_no-1)*total_px*4+(cam_no)*total_px,:)=[x_new y_new z_new];
           
        end
    end
    for cam_no=1:1:numCam
        camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
        save([camFolder '\closest_xpx_ypx_and_center_Z_selected68points.mat'],...
                        'x0_y0_z0_img','x0_y0_z0_real',"XYZcam_all","XYZworld_all")
    end
end
