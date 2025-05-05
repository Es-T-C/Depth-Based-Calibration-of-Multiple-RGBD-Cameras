%% functions/image2medianImage.m
function image2medianImage(folderPath, numImages, imageType, numIter,numCam)
% IMAGE2MEDIANIMAGE   Compute median depth images from multiple frames.
%   image2medianImage(folderPath, numImages, imageType, numIter,numCam)
%     folderPath : base folder path containing 'camN' subdirectories
%     numImages  : number of frames per measurement or position
%     imageType  : 1 = flat-wall images; 2 = tetrahedron object images
%     numIter    : number of measurements (flat-wall) or positions (object)
%     numCam     : number of calibrated cameras

    arguments
        folderPath (1,:) char
        numImages  (1,1) double {mustBePositive, mustBeInteger}
        imageType  (1,1) double {mustBeMember(imageType,[1,2])}
        numIter    (1,1) double {mustBePositive, mustBeInteger}
        numCam     (1,1) double {mustBePositive, mustBeInteger}
    end

    for cam_no = 1:numCam
        camFolder = fullfile(folderPath, sprintf('cam%d', cam_no));
        % Buffer to load frames
        A = zeros(480, 640, numImages, 'double');

        for iter_no = 1:numIter
            % Load each frame
            for img_no = 1:numImages
                switch imageType
                    case 1
                        fname = sprintf('depthImg%d_meas_%d.png', img_no, iter_no);
                    case 2
                        fname = sprintf('depthImg%dpos%d.png', img_no, iter_no);
                end
                I = imread(fullfile(camFolder, fname));
                A(:,:,img_no) = double(I);
            end

            % Replace zeros with NaN to ignore in median
            A(A==0) = nan;
            % Compute median across the third dimension
            M = median(A, 3, 'omitnan');
            % Replace NaNs with zero
            M(isnan(M)) = 0;

            % Write median image to disk
            switch imageType
                case 1
                    outName = sprintf('depthImg100_median_meas_%d.png', iter_no);
                case 2
                    outName = sprintf('depthImgMedian_pos%d.png', iter_no);
            end
            imwrite(uint16(M), fullfile(camFolder, outName));
        end
    end
end
