%% Finding global minimum with DE

% Test function: 2d Ackley's function
pop.func = @(x) -20*exp(-.2*sqrt(.5*sum(x.^2)))...
    -exp(.5*sum(cos(2*pi*x)))+exp(1)+20;
pop.bound = [-5,-5;5,5];

% Setup the DE settings
pop.size = 50; pop.life = 200; pop.type = 'DE';
result = DEMC(pop); 
disp(['Global minimum from DE: ' num2str(result.best)])

%% Mapping posterior with MCMC and DEMC

mu = [2 2];
sigma = [0.25 0.3; 0.3 1];
% y = mvnpdf(X,mu,sigma);


% Test function: wide spread mixture of two normal distributions: 1/3 of
% N(-5,1) and 2/3 of N(5,1). We expect a mean of 1.67 and a std of 4.82
% pop.func = @(x) (2/3)*normpdf(x,5,1)+(1/3)*normpdf(x,-5,1);
% pop.bound = [-100;100]+1.67;

pop.func = @(x) mvnpdf(x,mu,sigma);
pop.bound = [-5 -5;5 5];

% Using DEMC. It should only be able to sample both peaks.
pop.type = 'DEMC'; pop.life = 200;
result = DEMC(pop); burnt = result.chain(pop.life/2:end,:);
% disp(['Mean estimated by DEMC: ' num2str(mean(burnt(:)))])
% disp(['Std estimated by DEMC: ' num2str(std(burnt(:)))])

disp(['Maximum of function at: ' num2str(result.best)])



% Using MCMC. It should only be able to sample one peak.
% pop.life = 10000; pop.loc = 0;
% result = MCMC(pop); burnt = result.chain(pop.life/2:end,:);
% disp(['Mean estimated by MCMC: ' num2str(mean(burnt))])
% disp(['Std estimated by MCMC: ' num2str(std(burnt))])

