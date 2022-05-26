function[ts,pulse]=MYuwb_pulse(u)
%u=0;
fs=200E9;%sample frequency
ts=1/fs;%sample period
pulse_width=0.5E-9;%pulse width(0.5 nanoseconds),標準偏差（σ）

t=[(-4E-9+u):ts:(4E-9+u)];%vector with sample instants
x=((t-u)/pulse_width).^2/2;
pulse=exp(-x).*cos(2*pi*4E9.*t);%Gaussian pulse function
%pulse=awgn(pulse,10,1);

%1波長＝3*10^8/2*10^9＝15cm
%1/4波長＝3.75cm
%1秒で10回のパルス→時速10kmで移動したときのパルス発生間隔は27.777...cm

%figure(2)
%plot(1E9*t,pulse);%multiply t and y to get proper scaling and normalizing 
%hold on
%%plot(1E9*t,pulse,'o');
%xlabel('Time [ns]');ylabel('Amplitude');title('Gaussian pulse function')
