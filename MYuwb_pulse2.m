function[ts,pulse]=MYuwb_pulse2(u,u_len)
%u=randi(40,[1,41]); u_len=41;
fs=8E9;%sample frequency
ts=1/fs;%sample period
pulse_width=0.5E-9;%pulse width(0.5 nanoseconds),標準偏差（σ）
T=[-4E-9:ts:4E-9];
t=zeros(u_len,length(T));
x=zeros(u_len,length(T));
pulse=zeros(u_len,length(T));
for i=1:u_len
t(i,:)=[(-4E-9+u(1,i)):ts:(4E-9+u(1,i))];%vector with sample instants
x=((t(i,:)-u(1,i))/(pulse_width)).^2/2;
pulse(i,:)=exp(-x).*cos(2*pi*4.5E9.*t(i,:));%Gaussian pulse function
pulse(i,:)=pulse(i,:)/max(pulse(i,:));
end
%pulse=awgn(pulse,0,'measured');

%figure(1)
%plot(1E9*t,pulse);%multiply t and y to get proper scaling and normalizing 
%hold on
%plot(1E9*t,pulse,'o');
%xlabel('nanoseconds');ylabel('Amplitude');title('Gaussian pulse function')
