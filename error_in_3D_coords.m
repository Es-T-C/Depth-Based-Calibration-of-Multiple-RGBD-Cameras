%% functions/error_in_3D_coords.m
function error_in_3D_coords(objectFolder,numPos,numCam)
% error_in_3D_coords...   Calculate errors
%   error_in_3D_coords(objectFolder,numPos,numCam)
%     objectFolder : base folder path containing 'camN' subdirectories
%     numCam       : number of calibrated cameras
%     numPos       : number of positions (object)

    for method_no=0:1:3 % 0 -> for initials, 1 -> for BAin2D, 2->for BAin3D_M1, 3->for BAin3D_M2
    
        if method_no==0
            pt_title='initial';
        elseif method_no==1
            pt_title='BA_in2D';
        elseif method_no==2
            pt_title='BA_in3D_M1';
        else
            pt_title='BA_in3D_M2';
        end
        
        Wr=[373.2122 0 0]; %mm
        Wg=[-186.6061 0 -323.2113];%mm
        Wb=[-186.6061 0  323.2113];%mm
        Ww=[0 527.8018 0];%mm
        radius=10*(46/(2*pi)); %73.211;%mm
                
        E_all=zeros(4,numPos);E_all_std=zeros(4,numPos);
            for cam_no=1:1:numCam
                camFolder = fullfile(objectFolder, sprintf('cam%d', cam_no));
                for pos_no=1:1:numPos
                    pc=pcread([camFolder '\pos' int2str(pos_no) '_Params_' pt_title '.ply']);
                    dist_r=sqrt(sum((pc.Location-Wr).^2,2));
                    dist_g=sqrt(sum((pc.Location-Wg).^2,2));
                    dist_b=sqrt(sum((pc.Location-Wb).^2,2));
                    dist_w=sqrt(sum((pc.Location-Ww).^2,2));
                    Dist=[dist_r dist_g dist_b dist_w];
                    [min_dist, ~]=min(Dist,[],2);
                    E=abs(radius-min_dist);
                    E_mean=sum(E)/pc.Count;
                    E_std=std(E);
                    E_all(cam_no,pos_no)=E_mean;
                    E_all_std(cam_no,pos_no)=E_std;
                end
            end

            E_system_avg=sum(sum(E_all))/(size(E_all,1)*size(E_all,2));
            E_system_std_avg=sum(sum(E_all_std))/(size(E_all_std,1)*size(E_all_std,2));
        
            E_cam_avg=(sum((E_all'))/(numPos));
            E_cam_std_avg=(sum((E_all_std'))/(numPos));
            fprintf('     ->For %s parameters, the average system error : %0.4f mm\n',pt_title,E_system_avg)
    end
end
