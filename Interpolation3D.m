function pc_interp=Interpolation3D(pc,low,high)
xyz=round(pc.Location);


% idx{size(xyz),1}=[];
for i=1:1:size(xyz,1)
    dist=xyz-xyz(i,:).*ones(size(xyz,1),3);
    D=dist(:,1).^2+dist(:,2).^2+dist(:,3).^2;
    idx{i,1}=find(D>low^2 & D<high^2);
end
interpPoint=zeros(5*10^7,3);iter=0;
for i=1:1:size(xyz,1)
    for j=1:1:size(idx{i,1},1)
        iter=iter+1;
        interpPoint(iter,:)=[round((xyz(i,1)+xyz(idx{i,1}(j),1))/2) round((xyz(i,2)+xyz(idx{i,1}(j),2))/2) round((xyz(i,3)+xyz(idx{i,1}(j),3))/2) ];
    end
end

interpPoint2=interpPoint(1:iter,:);
[interpPoint3,~,~] = unique(interpPoint2,'rows','stable');

pc_interp=pointCloud(interpPoint3);

