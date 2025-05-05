function image_depthErrorRemoved=removeDepthError(image,I_LUT)

image_depthErrorRemoved=zeros(480,640);

for i=2:1:480
    for j=6:1:640
        depth_val=image(i,j);
        if depth_val<=2060 && depth_val>0
            image_depthErrorRemoved(i,j)=I_LUT(i,j,depth_val);
        else
            image_depthErrorRemoved(i,j)=depth_val;
        end
    end
end