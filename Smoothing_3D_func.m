function smooth_pc=Smoothing_3D_func(pc) 
xyz=round(pc.Location);
[xyz1,~,~] = unique(xyz,'rows','stable');
xyz_smooth1=zeros(size(xyz1,1),3);xyz_smooth2=zeros(size(xyz1,1),3);
for i=1:1:size(xyz1,1)
    dist=xyz1-xyz1(i,:).*ones(size(xyz1,1),3);
    D=(dist(:,1).^2+dist(:,2).^2+dist(:,3).^2);%Dist 2 selected point
    idx=find(D>0^2 & D<20^2);
    dist2point=D(idx);
    InR=xyz1(idx,:);
    Mx=mean(InR(:,1));    My=mean(InR(:,2));    Mz=mean(InR(:,3));
    dist2mean_tmp=InR-[Mx My Mz].*ones(size(InR,1),3);
    dist2mean=(dist2mean_tmp(:,1).^2+dist2mean_tmp(:,2).^2+dist2mean_tmp(:,3).^2);
    xyz_smooth1(i,:)=[Mx My Mz];
    idx2=find(dist2mean-dist2point>0);
    xyz_smooth2(i,:)=([mean(InR(idx2,1)) mean(InR(idx2,2)) mean(InR(idx2,3))]+[Mx My Mz])/2;
end
xyz_smooth=round([xyz_smooth1;xyz_smooth2]);
[xyz_smooth_uni,~,~] = unique(xyz_smooth,'rows','stable');
smooth_pc=pointCloud(xyz_smooth_uni);
