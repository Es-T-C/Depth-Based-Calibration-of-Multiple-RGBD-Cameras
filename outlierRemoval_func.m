function outRem_pc=outlierRemoval_func(pc,numNeighbour,thresh) 
xyz=round(pc.Location);
for i=1:1:size(xyz,1)
    dist_tmp=xyz-xyz(i,:).*ones(size(xyz,1),3);
    dist=(dist_tmp(:,1).^2+dist_tmp(:,2).^2+dist_tmp(:,3).^2);
    [~, idx]=mink(dist,numNeighbour+1);idx(1)=[];
    M=[mean(xyz(idx,1)) mean(xyz(idx,2)) mean(xyz(idx,3))];
    if norm(M-xyz(i,:))>thresh
        xyz(i,:)=[0 0 0];
    end
end
[xyz_uni,~,~] = unique(xyz,'rows','stable');
outRem_pc=pointCloud(xyz_uni);
