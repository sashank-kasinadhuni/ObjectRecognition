clc;clear;close all;
im1 = pcread('Image0.ply');
data = im1.Location(:,3);
data = mat2gray(transpose(reshape(data,640,480)));

Log = fspecial('log');
Laplacianofgaussian = imfilter(data,Log,'replicate');
keypoints = kp_log(data);

longitude = keypoints(:,2);
latitude = keypoints(:,1);

redImage = data;
greenImage = data;
blueImage = data;

for i = 1:size(keypoints,1)
    greenImage(keypoints(i,1),keypoints(i,2)) = 255;
    redImage(keypoints(i,1),keypoints(i,2)) = 0;
    blueImage(keypoints(i,1),keypoints(i,2)) = 0;
end
rgbImage = cat(3,redImage,greenImage,blueImage);
figure;
imshow(data);
hold on;
plot(longitude, latitude, 'r+', 'MarkerSize', 5, 'LineWidth', 3);
figure;
imshow(Laplacianofgaussian);
figure;
pcshow(im1);