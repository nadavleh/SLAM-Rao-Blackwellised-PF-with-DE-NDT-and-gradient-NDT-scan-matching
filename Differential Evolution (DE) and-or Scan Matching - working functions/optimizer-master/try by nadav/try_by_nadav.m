%% Finding global minimum with DE
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
k=200;
scans1(1:k)=[];
dt1(1:k)=[];
omometry_data1(1:k,:)=[];
X1(1:k,:)=[];




cur_scan = scan2cart(scans1{1,1}, 0); %let the current scan be the first one
cur_scan=cur_scan'; % transpose it inorder to tranform via J*cur_scan

alpha=pi/4; % we let the next scan be the same as the first but rotated pi/4 rad couner clock wise
t=[-3;3];   % and translated -3 to the left and +3 upwards

% the transformation matrix from the "next" crdnt system to the first
% coordinate system. in the next crdnt system, the scan seems in the same
% orientation as in the first one seems in its system however the robot has
% moved in "t" direction and rotated by "alpha"
J=[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)];
% and so, inorder to generate the next scan we simply totate it in the
% alpha direction and translate it by "t".
next_scan1=(J)*cur_scan+t;


% plot the two scans
scatter(cur_scan(1,:),cur_scan(2,:))
hold on
scatter(next_scan1(1,:),next_scan1(2,:))
axis square

%% now we need to match the scans and find t and alpha.

% generate the likelyhood pdf via the first scan
% empirical cov-matrix, can be changed with trial and error.
sigma=[0.8 0; 0 0.8]; 
% gmdistribution() doesnt like to get 2Xn crdnts matrix so we transpose to nX2
gm = gmdistribution(cur_scan',sigma);
% Check it works ok
vor = pdf(gm, cur_scan')+eps;

% generate a function which gets a transformation vector x=[t_x t_y alpha]
% then sends the next_scan to the original crdnt system specified by x, and
% evaluate its likelyhood

% % % % messy
% % % % % % % pop.func = @(x) pdf(gm, ( ([cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))])*next_scan1+[x(1);x(2)])');
% % % % Less messy
% % % % % % % J=@(x) [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
% % % % % % % pop.func = @(x) pdf(gm, ( (J(x))*next_scan1+[x(1);x(2)])');%+eps;

% even less messy
J=@(x) [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
trans=@(x) (J(x)^-1)*(next_scan1-[x(1);x(2)]);
pop.func = @(x) pdf(gm, trans(x)')+eps;

% we know the exact transformation we subjected the first scan inorder to
% transform it to the next scan, so lets check we get the currect results
% as aplying the first scan to the likelyhood func.
known_transform_vec=[t' alpha];
% see how many hits we got
sum(pop.func(known_transform_vec)== vor)

%% And another check with plots
known_transform_vec=[t' alpha ];
% known_transform_vec=[-0.01 4.25 alpha ];
J=@(x) [cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))];
trans=@(x) (J(x)^-1)*(next_scan1-[x(1);x(2)]);

next_scan2=trans(known_transform_vec);
% % sum(next_scan2(:)==cur_scan(:))
% only about 204 exact matches due to truncation errors:
sum(next_scan2(2,next_scan2(1,:)==cur_scan(1,:))==cur_scan(2,next_scan2(1,:)==cur_scan(1,:)))



scatter(cur_scan(1,:),cur_scan(2,:),'b')
hold on
scatter(next_scan2(1,:),next_scan2(2,:),'*r')
hold on
scatter(next_scan1(1,:),next_scan1(2,:),'g')
hold off
axis square

legend('original scan','scan transformed back','next scan (covering original scan)')


%% Now perform DE 
% set the likelyhood function again, now as a scalar function with sum()
pop.func = @(x) sum( pdf(gm, ( ([cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))]^-1)*(next_scan1-[x(1);x(2)]) )' )+eps);
% define the search bounds
pop.bound = [-3.1,  -3.1,   pi/4-0.1   ;...
              3.1,   3.1,  pi/4+0.1];
% define hyper parameters          
pop.size = 50; pop.life = 50; pop.type = 'DE';
% start DE algorithm on the function
result = DEMC(pop); 
disp(['Global minimum from DE: ' num2str(result.best)])

%%  perform DEMC 
% set the likelyhood function again, now as a scalar function with sum()
pop.func = @(x) sum( pdf(gm, ( ([cos(x(3)) -sin(x(3));sin(x(3)) cos(x(3))]^-1)*(next_scan1-[x(1);x(2)]) )' )+eps);
% define the search bounds
pop.bound = [-3.1,  -3.1,   pi/4-0.1   ;...
              3.1,   3.1,  pi/4+0.1];
% Using DEMC. It should only be able to sample both peaks.
pop.type = 'DEMC'; pop.life = 50;
result = DEMC(pop); burnt = result.chain(pop.life/2:end,:);
% disp(['Mean estimated by DEMC: ' num2str(mean(burnt(:)))])
% disp(['Std estimated by DEMC: ' num2str(std(burnt(:)))])

disp(['Maximum of function at: ' num2str(result.best)])

