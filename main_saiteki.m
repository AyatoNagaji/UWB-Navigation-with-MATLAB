%歩行ルート＆パルス発生位置
[x123,y123,len,step]=MYpulse_route2();

%センサーの位置
sensor1_x=9.12; sensor2_x=21.16; sensor3_x=23.79;
sensor1_y=1.94; sensor2_y=6.66; sensor3_y=14.76;
sensor4_x=15.14; sensor5_x=4.43;
sensor4_y=23.55; sensor5_y=19;
sensor=[sensor1_x sensor2_x sensor3_x sensor4_x sensor5_x;...
        sensor1_y sensor2_y sensor3_y sensor4_y sensor5_y];

d_sensor1=zeros(1,len); d_sensor2=zeros(1,len); d_sensor3=zeros(1,len);d_sensor4=zeros(1,len);
d_sensor5=zeros(1,len);

c=3.0e+8;   %光速
%センサーと送信機の距離を求める
d_sensor1=sqrt((x123-sensor1_x).^2+(y123-sensor1_y).^2);
d_sensor2=sqrt((x123-sensor2_x).^2+(y123-sensor2_y).^2);
d_sensor3=sqrt((x123-sensor3_x).^2+(y123-sensor3_y).^2);
d_sensor4=sqrt((x123-sensor4_x).^2+(y123-sensor4_y).^2);
d_sensor5=sqrt((x123-sensor5_x).^2+(y123-sensor5_y).^2);
d_sensor=[d_sensor1;d_sensor2;d_sensor3;d_sensor4;d_sensor5];

%送信機からセンサーまでの実際の到着時間を求める
t1_real=d_sensor1./c; t2_real=d_sensor2./c; t3_real=d_sensor3./c;
t4_real=d_sensor4./c; t5_real=d_sensor5./c;
t_real=[t1_real;t2_real;t3_real;t4_real;t5_real];

%チャネル応答などを経て、誤差が生じた到着時間を求める
t1=MYtimeerror2(3,20,t1_real,len);
t2=MYtimeerror2(3,20,t2_real,len);
t3=MYtimeerror2(3,20,t3_real,len);
t4=MYtimeerror2(3,20,t4_real,len);
t5=MYtimeerror2(3,20,t5_real,len);
t=[t1;t2;t3;t4;t5];

%傾き計算
slope_real=([d_sensor1 0]-[0 d_sensor1]);
slope_real=slope_real(1,2:len);
slope_t1=([t1 0]-[0 t1]).*c;
slope_t1=slope_t1(1,2:len);
slope_t2=([t2 0]-[0 t2]).*c;
slope_t2=slope_t2(1,2:len);
slope_t3=([t3 0]-[0 t3]).*c;
slope_t3=slope_t3(1,2:len);
slope_t4=([t4 0]-[0 t4]).*c;
slope_t4=slope_t4(1,2:len);
slope_t5=([t5 0]-[0 t5]).*c;
slope_t5=slope_t5(1,2:len);

%step(=時速10㎞で走った時の幅)を超えるインデックスを表示
step=step*2; %時速5kmに2をかけて10にするため.
notice1=find(slope_t1>step)+1;
notice2=find(slope_t2>step)+1;
notice3=find(slope_t3>step)+1;
notice4=find(slope_t4>step)+1;
notice5=find(slope_t5>step)+1;

%stepを超えるインデックスを１まとめに
len_notice1=length(notice1);
len_notice2=length(notice2);
len_notice3=length(notice3);
len_notice4=length(notice4);
len_notice5=length(notice5);
len_notice=max([len_notice1 len_notice2 len_notice3 len_notice4 len_notice5]);
notice=[[notice1 zeros(1,len_notice-len_notice1)];[notice2 zeros(1,len_notice-len_notice2)];...
    [notice3 zeros(1,len_notice-len_notice3)];[notice4 zeros(1,len_notice-len_notice4)];...
    [notice5 zeros(1,len_notice-len_notice5)]];
    

%各センサーごとにstepを超えたインデックスを記憶
remember_sensor=zeros(5,len);
for i=1:5
for j=1:len_notice
    a=notice(i,:);
    a=a(a~=0);
    remember_sensor(i,a)= 1;
end
end

%各インデックスごとの使えるセンサー数を計算
sensors=(5-sum(remember_sensor(1:5,:)));
%各インデックスごと使えるセンサー数でパターン分け
pattern1=find(sensors>2);
pattern2=find(sensors==2);
pattern3=find(sensors==1);
pattern4=find(sensors==0);

x=zeros(1,len); y=zeros(1,len);
%====================================== パターン1の場合 ======================================================
counter1=zeros(5,len); t_saiteki=zeros(3,len); sensor_saiteki=zeros(6,len);
for i=pattern1
counter1(1:sensors(1,i),i)=find(remember_sensor(:,i)==0);
t_saiteki(:,i)=t(counter1(1:3,i),i);
sensor_saiteki(1:2:6,i)=sensor(1,counter1(1:3,i));
sensor_saiteki(2:2:6,i)=sensor(2,counter1(1:3,i));
%測位
[x(1,i),y(1,i)]=MYTOA(t_saiteki(1,i),t_saiteki(2,i),t_saiteki(3,i),sensor_saiteki(1,i),sensor_saiteki(2,i),sensor_saiteki(3,i),sensor_saiteki(4,i),sensor_saiteki(5,i),sensor_saiteki(6,i));
end
%============================================================================================================


%================================= パターン2の場合 ==============================================================
counter2=zeros(5,len); t_saiteki2=zeros(2,len); sensor_saitek2=zeros(4,len);
xout=zeros(2,len); yout=zeros(2,len);
for i=pattern2
counter2(1:sensors(1,i),i)=find(remember_sensor(:,i)==0);
t_saiteki2(:,i)=t(counter2(1:2,i),i);
sensor_saiteki2(1:2:4,i)=sensor(1,counter2(1:2,i));
sensor_saiteki2(2:2:4,i)=sensor(2,counter2(1:2,i));
[xout(1:2,i),yout(1:2,i)]=circcirc(sensor_saiteki2(1,i),sensor_saiteki2(2,i),c*t(counter2(1,i),i),...
    sensor_saiteki2(3,i),sensor_saiteki2(4,i),c*t(counter2(2,i),i));
end

%%それぞれの交点と前回の良い測位点との距離
%d_prepoint1=sqrt((x(1,pattern2)-x_1(1,2:2:len)).^2+(y(1,1:2:len)-y_1(1,2:2:len)).^2);
%d_prepoint2=sqrt((x(1,1:2:len)-x_2(1,2:2:len)).^2+(y(1,1:2:len)-y_2(1,2:2:len)).^2);
%%距離が近いほうの交点を採用
%I=find((d_prepoint1-d_prepoint2)<0);
%J=find((d_prepoint1-d_prepoint2)>0);
%x(1,2*I)=x_1(1,2*I); y(1,2*I)=y_1(1,2*I);
%x(1,2*J)=x_2(1,2*J); y(1,2*J)=y_2(1,2*J);
%==============================================================================================


%%測位
%for i=1:len
 %   [x(1,i),y(1,i)]=MYTOA(t1(1,i),t2(1,i),t3(1,i),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
%end


%誤差を求める
d_error=zeros(1,length(pattern1));
d_error=sqrt((x123(1,pattern1)-x(1,pattern1)).^2+(y123(1,pattern1)-y(pattern1)).^2);
d_error_ave=sum(d_error)/length(pattern1) ;%誤差の平均


figure(2) %表示
stem(d_error); 
hold on
title('各点の誤差');
plot(len/2,d_error_ave,'rx'); ylabel('[m]')
hold off

figure(1) 
hold on
%　測位場所とセンサーの配置表示
plot(x(1,pattern1),y(1,pattern1),'ro'); 
plot(sensor1_x,sensor1_y,'bs'); plot(sensor2_x,sensor2_y,'bs'); 
plot(sensor3_x,sensor3_y,'bs'); plot(sensor4_x,sensor4_y,'bs');
plot(sensor5_x,sensor5_y,'bs');
%　歩行ルートの表示
plot(x123,y123,'g-');
%　パルス発生位置の表示
plot(x123,y123,'b+');
%枠
rectangle('Position',[ -3 -3 31 31]);
xlabel('[m]')
ylabel('[m]')
hold off

figure(3)
plot(d_sensor4);
hold on
plot(t4.*c); plot(width,lower3,'k');
title('センサーと発信源の距離')
hold off