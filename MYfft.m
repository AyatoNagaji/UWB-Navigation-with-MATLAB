fs=1000;
dt=1/fs;
t=(1:1024)*dt-dt;
s1=0.8*sin(2*pi*t*5); s2=0.9*sin(2*pi*t*250);
s3=0.5*sin(2*pi*t*450);
s=s1+s2+s3+rand(size(s1));


figure(1)
subplot(2,1,1);plot(t(1:100),s(1:100),'b');
xlabel('time[sec]'); ylabel('Amplitude'); title('data with noise');

power=abs(fft(s))/(length(s)/2);
nyquist=fs/2;
f=t/dt/length(t)*fs;
subplot(2,1,2);
index=find(diff(power)>0.4)+1;
plot(f,power,'b',f(index),power(index),'rp'); xlim([0 nyquist]);
xlabel('frequency[Hz]'); ylabel('Amplitude'); title('FFT result lenear plot');
