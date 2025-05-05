%% functions/FindCenterOfClosest.m
function [X0_calc, Y0_calc, depth_min1, depth_min2, levels] = FindCenterOfClosest(temp1, temp2, Num_img)

    t = linspace(0,2*pi,2000);
    % count occurrences of each depth level
    for k = 800:4094
        counter = 0;
        for img_no = 1:Num_img
            for i=1:length(temp1{img_no})
                if temp1{img_no}(i)==k
                    counter = counter + 1;
                else
                    counter=counter;
                end
            end
        end
        count_temp(k) = counter;
    end
    count_mask = count_temp>0;
    levels     = find(count_mask);

    if     numel(levels)>15, levels_remove = 6;
    elseif numel(levels)>10, levels_remove = 4;
    else                     levels_remove = 0;
    end

    % fit ellipses to each valid level
    l_s_axis = []; X0_Y0 = []; levels_new = [];
    for i = 1:(numel(levels)-levels_remove)
        coord_r_all{i} = []; coord_c_all{i} = [];
        for img_no = 1:Num_img
            I_tmp = temp2{img_no}(:,:);
            I_tmp(I_tmp<levels(i))=0;I_tmp(I_tmp>levels(i))=0;
            se   = strel('disk',5);
            closed{img_no}(:,:) = imclose(I_tmp, se);
            [r,c] = find(closed{img_no}(:,:)==levels(i));
            
            coord_r_all{i} = [coord_r_all{i}; r];
            coord_c_all{i} = [coord_c_all{i}; c];
        end
        % ellipse fit
        ell = fit_ellipse(coord_c_all{i}, coord_r_all{i});
        if ~isempty(ell) && ell.status~="Hyperbola found" && ell.long_axis <= 1.25*ell.short_axis
            x_fit_rot = ell.a*cos(t)+ell.X0 ;
            y_fit_rot = ell.b*sin(t)+ell.Y0 ;
            R = [cos(-1*ell.phi), -sin(-1*ell.phi);
                 sin(-1*ell.phi), cos(-1*ell.phi)];
            
            XY_rot_fit=R*[x_fit_rot;y_fit_rot];

            l_s_axis   = [l_s_axis [ell.long_axis; ell.short_axis]];
            X0_Y0      = [X0_Y0 R*[ell.X0; ell.Y0]];  % rotated back
            levels_new = [levels_new levels(i)];
        end
    end

    % quadratic fit to axes vs. depth
    levels_added = levels_new(1)-50:0.1:levels_new(end);
    [f1,~]   = createFit(levels_new, l_s_axis(1,:));
    xnorm    = (levels_added-mean(levels_new))/std(levels_new);
    y1       = f1.p1*xnorm.^2 + f1.p2*xnorm + f1.p3;
    [~,i1]   = min(abs(y1));
    depth_min1 = levels_added(i1);

    [f2,~]   = createFit(levels_new, l_s_axis(2,:));
    y2       = f2.p1*xnorm.^2 + f2.p2*xnorm + f2.p3;
    [~,i2]   = min(abs(y2));
    depth_min2 = levels_added(i2);

    depth_max = max(depth_min1, depth_min2);

    % center coordinates at depth_max
    [f3,~] = createFit(levels_new, X0_Y0(1,:));
    y3     = f3.p1*xnorm.^2 + f3.p2*xnorm + f3.p3;
    X0_calc=y3(levels_added==depth_max);

    [f4,~] = createFit(levels_new, X0_Y0(2,:));
    y4     = f4.p1*xnorm.^2 + f4.p2*xnorm + f4.p3;
    Y0_calc=y4(levels_added==depth_max);
end
