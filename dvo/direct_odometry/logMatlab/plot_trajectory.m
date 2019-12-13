clear all;
close all;

figure
%[timestamp,tx,ty,tz] = textread('gtLog.txt','%f %f %f %f'); 
[gt_time,tx,ty,tz,qx,qy,qz,qw] = textread('/home_local/font_al/datasets/rgbd_dataset_freiburg2_xyz/groundtruth_without_headers.txt','%f %f %f %f %f %f %f %f'); 
plot3(tx,ty,tz,'.r');
hold on;
[timestamp,tx,ty,tz] = textread('gtLog.txt','%f %f %f %f'); 
plot3(tx,ty,tz,'*b');