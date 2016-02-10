clc;clear;close all;
im1 = pcread('speaker.ply');
data = im1.Location(:,3);
minval = min(data);
maxval = max(data);
data8bit = mat2gray(transpose(reshape(data,640,480)));

data2 = transpose(reshape(data,640,480));
data13bit = zeros(480,640);

for i = 1:480
    for j= 1:640
        if(isnan(data2(i,j)))
            data13bit(i,j) = 0;
        else    
            data13bit(i,j) = data2(i,j)*(8192/maxval);
        end
    end
end

keypoints = kp_log(data13bit);

longitude = keypoints(:,2);
latitude = keypoints(:,1);

redImage = data13bit;
greenImage = data13bit;
blueImage = data13bit;

for i = 1:size(keypoints,1)
    greenImage(keypoints(i,1),keypoints(i,2)) = 255;
    redImage(keypoints(i,1),keypoints(i,2)) = 0;
    blueImage(keypoints(i,1),keypoints(i,2)) = 0;
end
rgbImage = cat(3,redImage,greenImage,blueImage);

figure;
imshow(data13bit,[0,8192]);
title('keypoints with 13 bit depth data');
hold on;
plot(longitude, latitude, 'r+', 'MarkerSize', 5, 'LineWidth', 3);
hold off;

for sigma = 2:15
    Log = fspecial('log',sigma*6+1,sigma);
    Laplacianofgaussian = imfilter(data13bit,Log,'replicate');
    figure;
    imshow(Laplacianofgaussian);
end

figure;
pcshow(im1);