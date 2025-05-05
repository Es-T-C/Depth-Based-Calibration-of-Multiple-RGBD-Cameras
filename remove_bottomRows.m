function [removed_img,der_img_last5row]=remove_bottomRows(img)
[r, c]=size(img);
img2=(img(round(r/2):end,:));
der_img=((img2(2:end,:)-img2(1:end-1,:)));
row_nums=zeros(1,32);
for k=1:1:32
    temp=img2(:,round(c/2)-16+(k-1))';
    row_nums(k)=max(find(temp==max(temp(temp<4098))));
end
row2cut=max(row_nums);
removed_img=img(1:end-(size(img2,1)-row2cut),:);
der_img=der_img(1:end-(size(img2,1)-row2cut),:);
der_img_last5row=der_img(end-5:end,:);
der_img_last5row(der_img_last5row>0)=1;
removed_img(end-5:end,:)=removed_img(end-5:end,:).*der_img_last5row;

