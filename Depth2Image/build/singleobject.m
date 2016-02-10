clc;clear;close all;
input_image = pcread('speaker.ply');
depth_data = input_image.Location(:,3);
organized_depth_data = transpose(reshape(depth_data,640,480));

speaker_depth_data = organized_depth_data(206:360,216:351);

speaker_mat_gray = mat2gray(speaker_depth_data);
depth_speaker_13_bit = zeros(size(speaker_depth_data,1),size(speaker_depth_data,2));
maxval = max(speaker_depth_data(:));
minval = min(speaker_depth_data(:));
for i = 1:size(speaker_depth_data,1)
    for j=1:size(speaker_depth_data,2)
        if(isnan(speaker_depth_data(i,j)))
            depth_speaker_13_bit(i,j) = 255;
        else
            depth_speaker_13_bit(i,j) = (speaker_depth_data(i,j)-minval)*(255/(maxval-minval));
        end
    end
end
figure;
title('before equalization');
imshow(depth_speaker_13_bit,[0,255]);



% keypoints = kp_log(depth_speaker_13_bit);
% 
% longitude = keypoints(:,2);
% latitude = keypoints(:,1);
% 
% redImage = depth_speaker_13_bit;
% greenImage = depth_speaker_13_bit;
% blueImage = depth_speaker_13_bit;
% 
% for i = 1:size(keypoints,1)
%     greenImage(keypoints(i,1),keypoints(i,2)) = 255;
%     redImage(keypoints(i,1),keypoints(i,2)) = 0;
%     blueImage(keypoints(i,1),keypoints(i,2)) = 0;
% end
% rgbImage = cat(3,redImage,greenImage,blueImage);
% 
% figure;
% imshow(depth_speaker_13_bit,[0,255]);
% title('keypoints with 13 bit depth data');
% hold on;
% plot(longitude, latitude, 'r+', 'MarkerSize', 5, 'LineWidth', 3);
% hold off;
% 
for sigma = 0.2:0.5:3
    Log = fspecial('log',5,sigma);
    Laplacianofgaussian = imfilter(depth_speaker_13_bit,Log,'replicate');
    
    figure;
    imshow(Laplacianofgaussian);
end
figure;
pcshow(input_image);