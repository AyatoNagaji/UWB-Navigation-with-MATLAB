 %歩行ルート＆パルス発生位置
[x123,y123,len,step]=MYpulse_route2();

%センサーの位置
sensor1_x=9.12; sensor2_x=21.16; sensor3_x=23.79;
sensor1_y=1.94; sensor2_y=6.66; sensor3_y=14.76;
sensor4_x=15.14; sensor5_x=4.43;
sensor4_y=23.55; sensor5_y=19;

sensor2_x=sensor5_x;
sensor2_y=sensor5_y;


c=3.0e+8;   %光速
%センサーと送信機の距離を求める
d_sensor1=sqrt((x123-sensor1_x).^2+(y123-sensor1_y).^2);
d_sensor2=sqrt((x123-sensor2_x).^2+(y123-sensor2_y).^2);
d_sensor3=sqrt((x123-sensor3_x).^2+(y123-sensor3_y).^2);

%送信機からセンサーまでの実際の到着時間を求める
t1_real=d_sensor1./c; t2_real=d_sensor2./c; t3_real=d_sensor3./c;

%チャネル応答などを経て、誤差が生じた到着時間を求める
t1=MYtimeerror2(3,10,t1_real,len);
t2=MYtimeerror2(3,10,t2_real,len);
t3=MYtimeerror2(3,10,t3_real,len);

x=zeros(1,len); y=zeros(1,len);
%測位
parfor i=1:len
    [x(1,i),y(1,i)]=MYTOA(t1(1,i),t2(1,i),t3(1,i),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
end

%back=25; %何点戻るか
%t1_env=zeros(1,len-back); t2_env=zeros(1,len-back); t3_env=zeros(1,len-back);
%x_env=zeros(1,len-back); y_env=zeros(1,len-back);
%%包絡線での測位
%parfor i=1:len-back
%[upper,lower1]=envelope(t1(1,1:back+i),19,'peak');
%t1_env(1,i)=lower1(1,i);
%[upper,lower2]=envelope(t2(1,1:back+i),19,'peak');
%t2_env(1,i)=lower2(1,i);
%[upper,lower3]=envelope(t3(1,1:back+i),19,'peak');
%t3_env(1,i)=lower3(1,i);
%%測位
%[x_env(1,i),y_env(1,i)]=MYTOA(t1_env(1,i),t2_env(1,i),t3_env(1,i),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
%end

%%包絡線の個数に合うように削る
%x=x(1,1:len-back); y=y(1,1:len-back);
%x123=x123(1,1:len-back); y123=y123(1,1:len-back);

%kフレームにおける最小値を求める。
error=zeros(1,100); error2=zeros(1,100);
%for k=1:100
k=8;
M1=zeros(1,1+fix((len+1-k)/k)); I1=zeros(1,1+fix((len+1-k)/k));
M2=zeros(1,1+fix((len+1-k)/k)); I2=zeros(1,1+fix((len+1-k)/k));
M3=zeros(1,1+fix((len+1-k)/k)); I3=zeros(1,1+fix((len+1-k)/k));
t1_min=zeros(1,len); t2_min=zeros(1,len); t3_min=zeros(1,len);
for i=1:k:len+1-k
[M1(1,1+(i-1)/k),I1(1,1+(i-1)/k)]=min(t1(1,i:i-1+k));
I1(1,1+(i-1)/k)=I1(1,1+(i-1)/k)+(i-1);

[M2(1,1+(i-1)/k),I2(1,1+(i-1)/k)]=min(t2(1,i:i-1+k));
I2(1,1+(i-1)/k)=I2(1,1+(i-1)/k)+(i-1);

[M3(1,1+(i-1)/k),I3(1,1+(i-1)/k)]=min(t3(1,i:i-1+k));
I3(1,1+(i-1)/k)=I3(1,1+(i-1)/k)+(i-1);
end

for i=1:length(I1)-1
    t1_min(1,I1(1,i):I1(1,i+1))=(M1(1,i+1)-M1(1,i))/(I1(1,i+1)-I1(1,i)).*(I1(1,i):I1(1,i+1))+M1(1,i)-I1(1,i)*(M1(1,i+1)-M1(1,i))/(I1(1,i+1)-I1(1,i));
end
for i=1:length(I2)-1
    t2_min(1,I2(1,i):I2(1,i+1))=(M2(1,i+1)-M2(1,i))/(I2(1,i+1)-I2(1,i)).*(I2(1,i):I2(1,i+1))+M2(1,i)-I2(1,i)*(M2(1,i+1)-M2(1,i))/(I2(1,i+1)-I2(1,i));
end
for i=1:length(I3)-1
    t3_min(1,I3(1,i):I3(1,i+1))=(M3(1,i+1)-M3(1,i))/(I3(1,i+1)-I3(1,i)).*(I3(1,i):I3(1,i+1))+M3(1,i)-I3(1,i)*(M3(1,i+1)-M3(1,i))/(I3(1,i+1)-I3(1,i));
end

I1(I1==0)=[]; I2(I2==0)=[]; I3(I3==0)=[];
min_check=max([I1(1,1) I2(1,1) I3(1,1)]); max_check=min([I1(1,length(I1)) I2(1,length(I2)) I3(1,length(I3))]);

x_min=zeros(1,len); y_min=zeros(1,len);
%測位
parfor i=min_check:max_check
    [x_min(1,i),y_min(1,i)]=MYTOA(t1_min(1,i),t2_min(1,i),t3_min(1,i),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
end

%誤差を求める
d_error=sqrt((x123-x).^2+(y123-y).^2);
d_error_ave=sum(d_error)/len ;%誤差の平均
d_error_min=sqrt((x123(1,min_check:max_check)-x_min(1,min_check:max_check)).^2+(y123(1,min_check:max_check)-y_min(1,min_check:max_check)).^2);
d_error_min_ave=sum(d_error_min)/length(d_error_min); %誤差の平均
%error(1,k)=d_error_min_ave;
%error2(1,k)=error(1,k)+(2*k-2)/100*5/3.6;
%end

[G,min_k]=min(error);
[G2,min_k2]=min(error2);

figure(1)
hold on
%　測位場所とセンサーの配置表示
plot(x(1,min_check:max_check),y(1,min_check:max_check),'r.');
plot(x_min(1,min_check:max_check),y_min(1,min_check:max_check),'b.');
plot(sensor1_x,sensor1_y,'ks'); plot(sensor2_x,sensor2_y,'ks'); 
plot(sensor3_x,sensor3_y,'ks');
%　歩行ルートの表示
plot(x123,y123,'g-');
%枠
%rectangle('Position',[ -3 -3 31 31]);
xlabel('[m]','Fontsize',14); ylabel('[m]','Fontsize',14);
hold off

figure(2)
hold on
plot(d_sensor1(1,1:len),'k');
plot(t1(1,1:len).*c);
%plot(t1_env*c,'b');
%plot(I1,M1.*c,'o');
plot(I1(1,1):I1(1,length(I1)),t1_min(1,I1(1,1):I1(1,length(I1))).*c,'b');
xlabel('測距番号','Fontsize',15); ylabel('測距値[m]','Fontsize',15);
%title('送信局と受信局の距離')
hold off

figure(3)
hold on
plot(error); plot(min_k,G,'mp');
plot(error2); plot(min_k2,G2,'rp');
xlabel('kの値','Fontsize',16); ylabel('誤差[m]','Fontsize',16);
hold off