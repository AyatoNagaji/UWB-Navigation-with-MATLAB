function[t,h]=MYchannelmodel(cm_num)
%clear;
no_output_files = 1; % non-zero: avoids writing output files of continuous-time responses
num_channels = 1; % number of channel impulse responses to generate
%rng(12,'v5normal'); % initialize state of function for repeatability
%rng(12,'v5uniform'); % initialize state of function for repeatability
%cm_num = 4; % channel model number from 1 to 8
% get channel model params based on this channel model number
[Lam,lambda,Lmean,lambda_mode,lambda_1,lambda_2,beta,Gam,gamma_0,Kgamma, ...
sigma_cluster,nlos,gamma_rise,gamma_1,chi,m0,Km,sigma_m0,sigma_Km, ...
sfading_mode,m0_sp,std_shdw,kappa,fc,fs] = uwb_sv_params_15_4a( cm_num );

ts = 1/fs; % sampling frequency
% get a bunch of realizations (impulse responses)
[h_ct,t_ct,t0,np] = uwb_sv_model_ct_15_4a(Lam,lambda,Lmean,lambda_mode,lambda_1, ...
lambda_2,beta,Gam,gamma_0,Kgamma,sigma_cluster,nlos,gamma_rise,gamma_1, ...
chi,m0,Km,sigma_m0,sigma_Km,sfading_mode,m0_sp,std_shdw,num_channels,ts);
% now reduce continuous-time result to a discrete-time result
[hN,N] = uwb_sv_cnvrt_ct_15_4a( h_ct, t_ct, np, num_channels, ts );
if N > 1
h = resample(hN, 1, N); % decimate the columns of hN by factor N
else
h = hN;
end
% correct for 1/N scaling imposed by decimation
% h = h * N; % normalized below..
% prepare to add the frequency dependency
K = 1; % K = Ko*Co^2/(4*pi)^2/d^n
% since the K is a constant, and the effect will be removed after
% normalization, so the K is set to be 1
h_len = length(h(:,1));
if (cm_num == 1||cm_num == 2|| cm_num == 7||cm_num == 8||cm_num ==9)
[h]= uwb_sv_freq_depend_ct_15_4a(h,fc,fs,num_channels,kappa);
else
[h]= uwb_sv_freq_depend_ct_15_4a(h,fc,fs,num_channels,0);
end

t = [0:(h_len-1)]*ts*1e-9; % for use in computing excess & RMS delays
h=abs(h);
%h=h(:,randi([1 num_channels]));
%h=(h(:,1)+h(:,2)+h(:,3)+h(:,4)+h(:,5))/5;
h=h.';

%********************************************************************
% Testing and ploting
%********************************************************************
% channel energy
%channel_energy = sum(abs(h).^2);

figure(1); 
plot(t*1E9,h);
grid on
title('Impulse response realizations')
xlabel('Time (nS)')

%figure(2)
%plot(0,abs(h(1,:)),'o');

