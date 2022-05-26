%歩行ルート＆パルス発生位置
[x123,y123,len,step]=MYpulse_route2();

%センサーの位置
sensor1_x=14/3*sqrt(2); sensor2_x=14/3*sqrt(2)*3; sensor3_x=14/3*sqrt(2)*3;
sensor1_y=14/3*sqrt(2)*2; sensor2_y=14/3*sqrt(2); sensor3_y=14/3*sqrt(2)*3;
sensor4_x=14/3*sqrt(2); sensor5_x=14/3*sqrt(2)*2;
sensor4_y=14/3*sqrt(2)*3; sensor5_y=14/3*sqrt(2)*2;

d_sensor1=zeros(1,len); 
d_sensor2=zeros(1,len); 
d_sensor3=zeros(1,len);
d_sensor4=zeros(1,len);
d_sensor5=zeros(1,len);
x=zeros(1,len); y=zeros(1,len);
c=3.0e+8;   %光速
%センサーと送信機の距離を求める
d_sensor1=sqrt((x123-sensor1_x).^2+(y123-sensor1_y).^2);
d_sensor2=sqrt((x123-sensor2_x).^2+(y123-sensor2_y).^2);
d_sensor3=sqrt((x123-sensor3_x).^2+(y123-sensor3_y).^2);
d_sensor4=sqrt((x123-sensor4_x).^2+(y123-sensor4_y).^2);
d_sensor5=sqrt((x123-sensor5_x).^2+(y123-sensor5_y).^2);

%送信機からセンサーまでの実際の到着時間を求める
t1_real=d_sensor1./c; t2_real=d_sensor2./c; t3_real=d_sensor3./c;
t4_real=d_sensor4./c; t5_real=d_sensor5./c;

%チャネル応答などを経て、誤差が生じた到着時間を求める
t1=MYtimeerror2(3,20,t1_real,len);
t2=MYtimeerror2(3,20,t2_real,len);
t3=MYtimeerror2(3,20,t3_real,len);
%t4=MYtimeerror2(3,20,t4_real,len);
%t5=MYtimeerror2(3,20,t5_real,len);


%測位
for i=1:len
    [x(1,i),y(1,i)]=MYTOA(t1(1,i),t2(1,i),t3(1,i),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
end

%誤差を求める
d_error=zeros(1,len);
d_error=sqrt((x123-x).^2+(y123-y).^2);
d_error_ave=sum(d_error)/len ;%誤差の平均


figure(2) %表示
plot(d_error); 
hold on
title('各点の誤差');
plot(len/2,d_error_ave,'rx'); ylabel('[m]')
hold off

figure(1) 
hold on
%　測位場所とセンサーの配置表示
%plot(x,y,'ro'); 
plot(sensor1_x,sensor1_y,'bs'); 
plot(sensor2_x,sensor2_y,'bs'); plot(sensor3_x,sensor3_y,'bs');
%　歩行ルートの表示
plot(x123,y123,'g-');
%　パルス発生位置の表示
plot(x123,y123,'k+');
%枠
rectangle('Position',[ -3 -3 31 31]);
xlabel('[m]')
ylabel('[m]')
hold off

figure(3)
plot(d_sensor1);
hold on
plot(t1.*c); %plot(lower3,'k');
title('センサーと発信源の距離')
hold off