%% main.m

clc;
clearvars; close all;
fprintf('=== Depth Calibration Pipeline ===\n\n');

%% Stage 1: Calculate Lookup Tables (LUTs) for flat-wall depth error removal
fprintf('-> Stage 1: Generate LUTs from flat-wall images\n');
flatFolder = uigetdir(pwd, 'Select folder containing flat-wall depth images');
framesPerMeasurement = 100;
numMeasurements      = 25;
numPositions         = 8;
numCameras           = 4;
fprintf('\n   -> Median flat-wall images are creating...\n');
image2medianImage(flatFolder, framesPerMeasurement, 1, numMeasurements,numCameras);
fprintf('\n   -> LUTs are creating...\n');
FlatWall2LUTs(flatFolder,numCameras,numMeasurements);

%% Stage 2: Apply depth error removal to object images
fprintf('\n-> Stage 2: Create median and sphere-only images and remove depth error\n');
objFolder         = uigetdir(pwd, 'Select folder containing object depth images');
framesPerPosition = 100;
fprintf('\n   -> Median images are creating...\n');
image2medianImage(objFolder, framesPerPosition, 2, numPositions,numCameras);
fprintf('\n   -> Sphere-only images are creating...\n');
only_spheres(objFolder,numPositions,numCameras);

% Stage 2b: Apply depth error removal on raw and sphere-only images
fprintf('\n-> Stage 2b: Apply depth error removal on raw, sphere-only and sphere-only median images\n');
fprintf('\n   -> Depth error removal is applying -> raw images...\n');
removeDepthErrorApply(objFolder, flatFolder,numPositions,numCameras,framesPerPosition,1,[]);%raw images
fprintf('\n   -> Depth error removal is applying -> Sphere-only images...\n');
removeDepthErrorApply(objFolder, flatFolder,numPositions,numCameras,framesPerPosition,2,2);%sphereOnly images
fprintf('\n   -> Depth error removal is applying -> Sphere-only median images...\n');
removeDepthErrorApply(objFolder, flatFolder,numPositions,numCameras,framesPerPosition,2,1);%sphereOnly median images
fprintf('\n   -> Depth error removal is applying -> Flat-wall median images...\n');
removeDepthErrorApply(objFolder, flatFolder,numPositions,numCameras,framesPerPosition,3,[]);%flatwall median images

%% Stage 3: Detect sphere centers and compute initial calibration parameters
fprintf('\n-> Stage 3: Detect sphere centers and estimate initial calibration\n');
fprintf('\n   -> Sphere centers/closest points are detecting...\n');
detectClosestsWithEllipses(objFolder, flatFolder,numPositions,numCameras,framesPerPosition)

% Stage 3b: Compute initial calibration parameters
FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses(objFolder,0,numPositions,numCameras);

% Stage 3c: Refine sphere centers with point-cloud fitting
fprintf('\n   -> Sphere centers are refining...\n');
DetectSpherePC_WithInitialParams_onSphereImages(objFolder,numCameras,numPositions,false);
FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses(objFolder,1,numPositions,numCameras);
DetectSpherePC_WithInitialParams_onSphereImages(objFolder,numCameras,numPositions,true);
FindIntrinsicsExtrinsicsUsingSphereCentersFromEllipses(objFolder,1,numPositions,numCameras);

% Stage 4: Transform to world coordinates and sample cross-shaped points
fprintf('\n-> Stage 4: Transform to world coordinates and sample cross-shaped points\n');
fprintf('\n   -> Depth images are transforming into world coordinates...\n');
image2worldWithInitials(objFolder, numPositions,numCameras);
fprintf('\n   -> Cross-shaped points are sampling...\n');
world2imageSpherePointSampling(objFolder,numPositions,numCameras);

% Stage 5: Perform Bundle Adjustment
fprintf('\n-> Stage 5: Run bundle adjustment methods\n');
fprintf('\n   -> BA in 2D is processing...\n');
BA_In2D(objFolder,numPositions,numCameras);
fprintf('\n   -> BA in 3D Method 1 is processing...\n');
BA_In3D_M1(objFolder,numPositions,numCameras,1);
fprintf('\n   -> BA in 3D Method 2 is processing...\n');
BA_In3D_M2(objFolder,numPositions,numCameras,1);

% Stage 6: Reconstruct point cloud using BA parameters and compute error 
fprintf('\n-> Stage 6: Reconstruct point clouds and compute error\n');
fprintf('\n   -> Depth images are transforming into world coordinates...\n');
image2worldWithAllParams(objFolder,numPositions,numCameras);
fprintf('\n   -> Errors are computing...\n');
error_in_3D_coords(objFolder,numPositions,numCameras);

fprintf('\n=== Depth Calibration Pipeline Completed ===\n');
