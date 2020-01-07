% refRanges = 5*ones(1,300);
% refAngles = linspace(0,pi,300);
% cur_scan=[refRanges.*cos(refAngles);refRanges.*sin(refAngles)];
% 
% alpha=pi/4;
% J=[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)];
% 
% next_scan=(J)*cur_scan+[-3;3];
% 
% scatter(cur_scan(1,:),cur_scan(2,:))
% hold on
% scatter(next_scan(1,:),next_scan(2,:))
% axis square
%% Load Robot 1 data

clear;

load('robot1.mat');
dt1=cell2mat(dt1);
omometry_data1=cell2mat(omometry_data1);
X1=cell2mat(X1);

map_to_obsticale_vector;
real_obsticle_vector=obsticle_vector;

clearvars -except  dt1 omometry_data1 X1 scans1 real_obsticle_vector

clc;
%% discard k first entries because robot is static at the begining
k=40;
scans1(1:k)=[];
dt1(1:k)=[];
omometry_data1(1:k,:)=[];
X1(1:k,:)=[];