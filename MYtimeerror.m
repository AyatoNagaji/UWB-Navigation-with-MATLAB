%function[check_error]=MYerrortime(cm_num,awgn,u)
[h_t,h]=MYchannelmodel(3);

%h(1,1:length(h_t))=0; h(1,1)=1; %理想のチャネル応答

u=0E-9;
[ts,s]=MYuwb_pulse(u);
conv_len=length(s)+length(h)-1;
r=zeros(1,conv_len);
r=conv(s,h);
r=[zeros(1,fix(conv_len/2)) r(1,1:round(conv_len/2))];
[r]=MYawgn(r,10,50);

check=zeros(1,2*length(r)-1);
lags=zeros(1,2*length(r)-1);
[check,lags]=xcorr(s,r);
check=abs(check);


r_t=[(-4E-9)+u-fix(conv_len/2)*ts:ts:ts*(conv_len-1)-4E-9+u-fix(conv_len/2)*ts];

[m,n]=max(check);
[o,p]=max(r);
r_time=abs((r_t(1,p)))*1E9
check_time=(-lags(1,n)*ts+u-fix(conv_len/2)*ts)*1E9 %sとrが何サンプル数分ずれているか*ts+u [s]

figure(1)
subplot(3,1,2); plot(r_t*1E9,r); xlabel('time[ns]'); title('受信信号');
subplot(3,1,1); plot(h_t*1E9,h); xlabel('time[ns]'); title('チャネル応答');
subplot(3,1,3); plot(lags+round(conv_len/2),check); title('相互相関');