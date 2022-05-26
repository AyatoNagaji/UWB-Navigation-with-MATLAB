function[check_time]=MYtimeerror2(cm_num,snr,u,u_len)
%u=zeros(1,31); cm_num=3; snr=10; u_len=31;

[ts,s]=MYuwb_pulse2(u,u_len);

check_time=zeros(1,u_len);
parfor i=1:u_len
[h_t,h]=MYchannelmodel(cm_num);
%h(1,1:500)=0; h(1,2)=1; %理想のチャネル応答
conv_len=length(s(i,:))+length(h)-1;
r=conv(s(i,:),h);
r=[zeros(1,fix(conv_len/2)) r(1,1:round(conv_len/2))];
[r]=MYawgn(r,snr,50);

 %check=zeros(1,2*length(r)-1);
 %lags=zeros(1,2*length(r)-1);

[check,lags]=xcorr(s(i,:),r);

 %r_t=[-4E-9+u-fix(conv_len/2)*ts:ts:ts*(conv_len-1)-4E-9+u-fix(conv_len/2)*ts];

[m,n]=max(check);
 %[o,p]=max(r);
 %r_time=abs((r_t(1,p))); %受信信号の最大値

check_time(1,i)=-lags(1,n)*ts+u(1,i)-fix(conv_len/2)*ts; %sとrが何サンプル数分ずれているか*ts[s]
end

%figure(1)
%subplot(3,1,2); plot(r_t*1E9,r); xlabel('time[ns]'); title('受信信号');
%subplot(3,1,1); plot(h_t*1E9,h); xlabel('time[ns]'); title('チャネル応答(LOS)');
%subplot(3,1,3); plot(lags+round(conv_len/2),check); title('相互相関'); 