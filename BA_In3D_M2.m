%% functions/BA_In3D_M2.m
function BA_In3D_M2(objectFolder,numPos,numCam,enable_BA2Dparams)
% BA_In3D_M2...   Bundle‐adjust intrinsics/extrinsics by minimizing 3D reprojection error Method2
%   BA_In3D_M2(objectFolder,numPos,numCam,enable_BA2Dparams)
%     objectFolder : base folder path containing 'camN' subdirectories
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)
%     enable_BA2Dparams: % 1 -> load BAin2D params, 0 -> create new initial parameter vector
    numPoint=68;
    pixelsize=0.0104; 
    save('objectFolder.mat',"objectFolder");
    camFolder = fullfile(objectFolder, sprintf('cam%d', 1));
    load(fullfile(camFolder, 'closest_xpx_ypx_and_center_Z_selected68points.mat'))
    idx=find(x0_y0_z0_img(:,3)==0);
    x0_y0_z0_img(idx,3)=x0_y0_z0_real(idx,3);
    
    if enable_BA2Dparams==0
        for cam_no=1:numCam
            camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
            load(fullfile(camFolder, 'initialParams_FromProjectionMatrixMethodAndSphereFitting.mat'))
            k=[0 0];
            fx_cam{cam_no}=pixelsize*mean(fx_px);fy_cam{cam_no}=pixelsize*mean(fy_px);
            ox_cam{cam_no}=pixelsize*mean(ox_px);oy_cam{cam_no}=pixelsize*mean(oy_px);
            k1_cam{cam_no}=k(1);k2_cam{cam_no}=k(2);
            wx_cam_all{cam_no}=zeros(numPos);wy_cam_all{cam_no}=zeros(numPos);wz_cam_all{cam_no}=zeros(numPos);
            tx_cam_all{cam_no}=zeros(numPos);ty_cam_all{cam_no}=zeros(numPos);tz_cam_all{cam_no}=zeros(numPos);
            R{cam_no}=zeros(3,3,numPos);
            for pos_no=1:1:numPos
                [Q,R{cam_no}(:,:,pos_no)]=qr(R_new(:,:,pos_no));
                theta_temp=real(acos((trace(Q)-1)/2));
                w_temp=(1/(2*sin(theta_temp)))*[Q(3,2)-Q(2,3); Q(1,3)-Q(3,1); Q(2,1)-Q(1,2)];
                w_temp=w_temp/norm(w_temp);w_temp=theta_temp*w_temp;
                wx_cam_all{cam_no}(pos_no)=w_temp(1);wy_cam_all{cam_no}(pos_no)=w_temp(2);wz_cam_all{cam_no}(pos_no)=w_temp(3);
                tx_cam_all{cam_no}(pos_no)=T_new(1,1,pos_no);ty_cam_all{cam_no}(pos_no)=T_new(2,1,pos_no);tz_cam_all{cam_no}(pos_no)=T_new(3,1,pos_no);
            end
        end
        par=[];
        for cam_no=1:numCam
            for i=1:1:numPos
                par_extr_tmp{cam_no}((i-1)*6+1:i*6,:)=[wx_cam_all{cam_no}(i);wy_cam_all{cam_no}(i);wz_cam_all{cam_no}(i);tx_cam_all{cam_no}(i);ty_cam_all{cam_no}(i);tz_cam_all{cam_no}(i)];
            end
            par=[par;fx_cam{cam_no};fy_cam{cam_no};ox_cam{cam_no};oy_cam{cam_no};k1_cam{cam_no};k2_cam{cam_no};par_extr_tmp{cam_no}];
        end
        R1=R{1};R2=R{2};R3=R{3};R4=R{4};
        filename = 'rot_matrices_BAin3D_M2.mat';
        save(filename,'R1','R2','R3','R4')
    else
        par=load("params_BA_in2D.mat","p").p;
        load("rot_matrices_BAin2D.mat","R1","R2","R3","R4");
        filename = 'rot_matrices_BAin3D_M2.mat';
        save(filename,'R1','R2','R3','R4')
    end
    %%
    x = reshape(x0_y0_z0_img',[numPoint*3*numPos*numCam 1]);% 68points X 3axis X 8pos X 4cams
    y = zeros(length(x),1);
    options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt','Display','none','FunctionTolerance',10^-200);
    options.StepTolerance=10^-200;
    
    p = lsqcurvefit(@errFun, par(:,1), x, y,[],[],options);
    save("params_BA_in3D_M2.mat","p");
    
    e1=errFun(par(:,1),x);
    E1=sqrt(e1'*e1)/(numPos*numCam*numPoint);
    
    e2=errFun(p,x);
    E2=sqrt(e2'*e2)/(numPos*numCam*numPoint);
    perc=(E1-E2)*100/E1;
    disp(['     ->Error on world coords. is reduced ' int2str(perc) '%'])
    fprintf('     ->Error with 2D refined parameters : %0.4e mm\n',E1)
    fprintf('     ->Error with 3D_M2 refined parameters : %0.4e mm\n',E2)
end
%%
function E=errFun(p,x_m)
load('objectFolder.mat');
camFolder = fullfile(objectFolder, sprintf('cam%d', 1));
load(fullfile(camFolder, 'closest_xpx_ypx_and_center_Z_selected68points.mat'))
load('rot_matrices_BAin3D_M2.mat','R1','R2','R3','R4')
pixelsize=0.0104;
numPoint=68;
x0_y0_z0_img = reshape(x_m,[3 numPoint*32])';e=[];
for i=1:1:8
    for j=1:1:4
        if     j==1, R=R1(:,:,i);
        elseif j==2, R=R2(:,:,i);
        elseif j==3, R=R3(:,:,i);
        elseif j==4, R=R4(:,:,i);
        end
        p_cam=p(54*(j-1)+1:54*(j-1)+54,1);% 54 = (6extrinsic X 8pos) + 6intrinsic
        p_pos=[p_cam(1:6);p_cam(1+6*i:6+6*i)];
        x_mm=x0_y0_z0_img(numPoint*4*(i-1)+(j)*numPoint-67:numPoint*4*(i-1)+(j)*numPoint,1)*pixelsize;
        y_mm=x0_y0_z0_img(numPoint*4*(i-1)+(j)*numPoint-67:numPoint*4*(i-1)+(j)*numPoint,2)*pixelsize;
        z_mm=x0_y0_z0_img(numPoint*4*(i-1)+(j)*numPoint-67:numPoint*4*(i-1)+(j)*numPoint,3);

        x_world=XYZworld_all(12*(i-1)+(j)*3-2,:);
        y_world=XYZworld_all(12*(i-1)+(j)*3-1,:);
        z_world=XYZworld_all(12*(i-1)+(j)*3-0,:);

        fx=p_pos(1);fy=p_pos(2);ox=p_pos(3);oy=p_pos(4);k1=p_pos(5);k2=p_pos(6);
        wx=p_pos(7); wy=p_pos(8); wz=p_pos(9); tx=p_pos(10); ty=p_pos(11); tz=p_pos(12);
        for ii=1:1:numPoint
            e=[e; dxyz_function(R,x_mm(ii), y_mm(ii), z_mm(ii), fx, fy, ox, oy, k1, k2, wx, wy, wz, tx, ty, tz, x_world(ii), y_world(ii), z_world(ii))];
        end
    end
end
E=e;
end


function e = dxyz_function(R3x3,x_mm, y_mm, z_mm, fx, fy, ox, oy, ~, ~, wx, wy, wz, tx, ty, tz, ~, ~, ~)
theta=sqrt(wx^2+wy^2+wz^2);
w=[wx wy wz]/theta;
Q_new=[cos(theta)+w(1)*w(1)*(1-cos(theta))     ,  w(1)*w(2)*(1-cos(theta))-w(3)*sin(theta),  w(1)*w(3)*(1-cos(theta))+w(2)*sin(theta);
       w(2)*w(1)*(1-cos(theta))+w(3)*sin(theta),  cos(theta)+w(2)*w(2)*(1-cos(theta))     ,  w(2)*w(3)*(1-cos(theta))-w(1)*sin(theta);
       w(3)*w(1)*(1-cos(theta))-w(2)*sin(theta),  w(3)*w(2)*(1-cos(theta))+w(1)*sin(theta),  cos(theta)+w(3)*w(3)*(1-cos(theta))];
Rot=Q_new*R3x3;
X_prime_m=(x_mm-ox)/-fx;
Y_prime_m=(y_mm-oy)/-fy;
XYZcamera_m=[X_prime_m*z_mm;Y_prime_m*z_mm;z_mm];
XYZworld_m=Rot\(XYZcamera_m-[tx;ty;tz]);
radius=10*(46/(2*pi));%mm
Pw=[XYZworld_m(1) XYZworld_m(2) XYZworld_m(3)];
R=[373.2122 0 0];
G=[-186.6061 0 -323.2113];
B=[-186.6061 0 323.2113];
W=[0 527.8018 0];
Cw=[R;G;B;W];
dist=[norm(R-Pw) norm(G-Pw) norm(B-Pw) norm(W-Pw)];
[~, b]=min(dist);
center=Cw(b,:);
d_CP=norm(Pw-center);
if d_CP>radius
    Pr_x=((XYZworld_m(1)-center(1))*radius/d_CP)+center(1);
    Pr_y=((XYZworld_m(2)-center(2))*radius/d_CP)+center(2);
    Pr_z=((XYZworld_m(3)-center(3))*radius/d_CP)+center(3);
elseif d_CP<radius
    Pr_x=((XYZworld_m(1)-center(1))*(radius-d_CP)/d_CP)+XYZworld_m(1);
    Pr_y=((XYZworld_m(2)-center(2))*(radius-d_CP)/d_CP)+XYZworld_m(2);
    Pr_z=((XYZworld_m(3)-center(3))*(radius-d_CP)/d_CP)+XYZworld_m(3);
end
e=[XYZworld_m(1)-Pr_x;
   XYZworld_m(2)-Pr_y;
   XYZworld_m(3)-Pr_z];
end