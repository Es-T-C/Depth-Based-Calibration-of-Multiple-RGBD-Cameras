function [R,T,xyzPoints]=Pw_Pc_forCenters (Im_rc,ox_avg,oy_avg,fx_avg,fy_avg,A_allPos,Z_RGBW_center,M_lastRow,Num_pos)

[row, ~]=size(Im_rc);
pixelsize=0.0104;
i=0;
for pos_no=1:1:Num_pos
    for k=1:1:row
        i=i+1;
        xyzPoints(i,1,pos_no)=((Im_rc(k,1,pos_no)*pixelsize-ox_avg)*Z_RGBW_center(k,:,pos_no))/-fx_avg;
        xyzPoints(i,2,pos_no)=((Im_rc(k,2,pos_no)*pixelsize-oy_avg)*Z_RGBW_center(k,:,pos_no))/-fy_avg;
        xyzPoints(i,3,pos_no)= Z_RGBW_center(k,:,pos_no);
    end
    i=0;
    b(:,:,pos_no)=[xyzPoints(1,1,pos_no);xyzPoints(1,2,pos_no);xyzPoints(2,1,pos_no);xyzPoints(2,2,pos_no);...
                xyzPoints(3,1,pos_no);xyzPoints(3,2,pos_no);xyzPoints(4,1,pos_no);xyzPoints(4,2,pos_no)];
end

for pos_no=1:1:Num_pos
    m(:,:,pos_no)=A_allPos(:,:,pos_no)\b(:,:,pos_no);
    R(:,:,pos_no)=[m(1:3,1,pos_no)';m(5:7,1,pos_no)';(M_lastRow(1:3,pos_no)*pixelsize)'];
    T(:,:,pos_no)=[m(4,1,pos_no);m(8,1,pos_no);M_lastRow(4,pos_no)*pixelsize];
end
