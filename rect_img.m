% Bouguet, J. (2001). Camera calibration toolbox for matlab.
% (https://robots.stanford.edu/cs223b04/JeanYvesCalib/)
function  [I_undistorted,x_map,y_map]=rect_img(f,c,k,I)
KK_new = [f(1) 0 c(1);0 f(2) c(2);0 0 1];
[nr,nc] = size(I);
I_rec = zeros(nr,nc); 

[mx,my] = meshgrid(1:1:nc, 1:1:nr);
px = reshape(mx',nr*nc,1);
py = reshape(my',nr*nc,1);
temp=[(px)';(py )';ones(1,length(px))];
rays = inv(KK_new)*temp;


% Rotation: (or affine transformation):

rays2 = eye(3)'*rays;

x = [rays2(1,:)./rays2(3,:);rays2(2,:)./rays2(3,:)];


% Add distortion:
xd = apply_distortion(x,k);


% Reconvert in pixels:

px2 = f(1)*xd(1,:)+c(1);
py2 = f(2)*xd(2,:)+c(2);


% Interpolate between the closest pixels:

px_0 = round(px2);


py_0 = round(py2);

good_points = find((px_0 > 0) & (px_0 <= (nc)) & (py_0 > 0) & (py_0 <= (nr)));

px_0 = px_0(good_points);
py_0 = py_0(good_points);

px = px(good_points);
py = py(good_points);

px2 = px2(good_points);
py2 = py2(good_points);

x_map=[px2;px'];
y_map=[py2;py'];

for i=1:1:length(good_points)
    I_rec(round(py(i)),round(px(i)))=I(py_0(i),px_0(i));
end
I_undistorted=I_rec;


