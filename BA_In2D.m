%% functions/BA_In2D.m
function BA_In2D(objectFolder,numPos,numCam)
% BA_In2D...   Bundle‐adjust intrinsics/extrinsics by minimizing 2D reprojection error
%   BA_In2D(objectFolder,numPos,numCam)
%     objectFolder : base folder path containing 'camN' subdirectories
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)

pixelsize=0.0104; %mm/px

xr=zeros(4,numPos);xg=zeros(4,numPos);xb=zeros(4,numPos);xw=zeros(4,numPos);
yr=zeros(4,numPos);yg=zeros(4,numPos);yb=zeros(4,numPos);yw=zeros(4,numPos);
x0_y0_z0_img=[];
for cam_no=1:1:numCam
    camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
    X0_Y0_Z0_calc_img_cam=load(fullfile(camFolder, 'center_xpx_ypx_Z.mat')).X0_Y0_Z0 ;
    for pos_no=1:1:numPos
        xr(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,1);yr(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,5);
        xg(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,2);yg(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,6);
        xb(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,3);yb(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,7);
        xw(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,4);yw(cam_no,pos_no)=X0_Y0_Z0_calc_img_cam((pos_no-1)*4+cam_no,8);
    end
end
for pos_no=1:1:numPos
    for cam_no=1:1:4
        x0_y0_z0_img=[x0_y0_z0_img;
                      xr(cam_no,pos_no),yr(cam_no,pos_no);xg(cam_no,pos_no),yg(cam_no,pos_no);
                      xb(cam_no,pos_no),yb(cam_no,pos_no);xw(cam_no,pos_no),yw(cam_no,pos_no)];
    end
end
X_R=373.2122; Y_R=0; Z_R=0;
X_G=-186.6061; Y_G=0; Z_G=-323.2113;
X_B=-186.6061; Y_B=0; Z_B=323.2113;
X_W=0; Y_W=527.8018; Z_W=0;
XYZworld_all=[X_R X_G X_B X_W; Y_R Y_G Y_B Y_W;Z_R Z_G Z_B Z_W];

for cam_no=1:numCam
    camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
    load(fullfile(camFolder, 'initialParams_FromProjectionMatrixMethodAndSphereFitting.mat'))
    load(fullfile(camFolder, 'distortion_k_fromCheckerboard.mat'),'k');
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
filename = 'rot_matrices_BAin2D.mat';
R1=R{1};R2=R{2};R3=R{3};R4=R{4};
save(filename,'R1','R2','R3','R4')
save('XYZworld_all.mat','XYZworld_all')

%%
x = reshape(x0_y0_z0_img',[numCam*2*4*numPos 1]);% 4cam X 2axis X 4 sphere X 8 pos
y = zeros(numCam*2*4*numPos,1); 
options = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt','Display','none','FunctionTolerance',10^-300);
options.StepTolerance=10^-300;
options.MaxFunctionEvaluations=10^200;
options.MaxIterations=10^200;
[p] = lsqcurvefit(@errFun, par(:,1), x, y,[],[],options);

save("params_BA_in2D.mat","p");
save("params_initial.mat","par");

e1=errFun(par(:,1),x);
E1=sqrt(e1'*e1)/(numPos*4*4);

e2=errFun(p,x);
E2=sqrt(e2'*e2)/(numPos*4*4);
perc=(E1-E2)*100/E1;
disp(['     ->Error on world coords. is reduced ' int2str(perc) '%'])
fprintf('     ->Error with initial parameters : %0.4e mm\n',E1)
fprintf('     ->Error with refined parameters : %0.4e mm\n',E2)

end
%%
function E=errFun(p,x_m)
load('rot_matrices_BAin2D.mat','R1','R2','R3','R4')
load('XYZworld_all.mat','XYZworld_all')
x0_y0_z0_img = reshape(x_m,[2 4*32])';
e=[];
pixelsize=0.0104;
for i=1:1:8
    for j=1:1:4
        if     j==1, R=R1(:,:,i);
        elseif j==2, R=R2(:,:,i);
        elseif j==3, R=R3(:,:,i);
        elseif j==4, R=R4(:,:,i);
        end
        p_cam=p(54*(j-1)+1:54*(j-1)+54,1); % 54 = (6extrinsic X 8pos) + 6intrinsic
        p_pos=[p_cam(1:6);p_cam(+1+6*i:6+6*i)];
        x_mm=x0_y0_z0_img(4*4*(i-1)+(j)*4-3:4*4*(i-1)+(j)*4,1)*pixelsize;
        y_mm=x0_y0_z0_img(4*4*(i-1)+(j)*4-3:4*4*(i-1)+(j)*4,2)*pixelsize;

        x_world=XYZworld_all(1,:);
        y_world=XYZworld_all(2,:);
        z_world=XYZworld_all(3,:);

        fx=p_pos(1);fy=p_pos(2);ox=p_pos(3);oy=p_pos(4); k1=p_pos(5);k2=p_pos(6);
        wx=p_pos(7); wy=p_pos(8); wz=p_pos(9); tx=p_pos(10); ty=p_pos(11); tz=p_pos(12);
        for ii=1:1:4
            e=[e; dxyz_function(R,x_mm(ii), y_mm(ii), fx, fy, ox, oy, k1, k2, wx, wy, wz, tx, ty, tz, x_world(ii), y_world(ii), z_world(ii))];
        end
    end
end
E=e;
end

function e = dxyz_function(R,x_mm, y_mm, fx, fy, ox, oy, k1, k2, wx, wy, wz, tx, ty, tz, Xw, Yw, Zw)
theta=sqrt(wx^2+wy^2+wz^2);
w=[wx wy wz]/theta;
Q_new=[cos(theta)+w(1)*w(1)*(1-cos(theta))     ,  w(1)*w(2)*(1-cos(theta))-w(3)*sin(theta),  w(1)*w(3)*(1-cos(theta))+w(2)*sin(theta);
       w(2)*w(1)*(1-cos(theta))+w(3)*sin(theta),  cos(theta)+w(2)*w(2)*(1-cos(theta))     ,  w(2)*w(3)*(1-cos(theta))-w(1)*sin(theta);
       w(3)*w(1)*(1-cos(theta))-w(2)*sin(theta),  w(3)*w(2)*(1-cos(theta))+w(1)*sin(theta),  cos(theta)+w(3)*w(3)*(1-cos(theta))];
Rot=Q_new*R;

XYZcamera=[Rot [tx;ty;tz]]*[Xw; Yw; Zw; 1];
X_prime=XYZcamera(1)/XYZcamera(3);
Y_prime=XYZcamera(2)/XYZcamera(3);
r=sqrt(X_prime^2+Y_prime^2);
X_prime_dist=X_prime*(1+k1*r^2+k2*r^4);
Y_prime_dist=Y_prime*(1+k1*r^2+k2*r^4);
x=-fx*X_prime_dist+ox;
y=-fy*Y_prime_dist+oy;

e=[x_mm-x;
   y_mm-y;
   ];
end

