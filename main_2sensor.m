%歩行ルート＆パルス発生位置
[x123,y123,len]=MYpulse_route2(); 

%センサーの位置
sensor1_x=14/3*sqrt(2); sensor2_x=14/3*sqrt(2)*3; sensor3_x=14/3*sqrt(2)*3;
sensor1_y=14/3*sqrt(2); sensor2_y=14/3*sqrt(2); sensor3_y=14/3*sqrt(2)*3;

d_sensor1=zeros(1,len); 
d_sensor2=zeros(1,len);
d_sensor3=zeros(1,len);
%センサーと送信機の距離を求める
d_sensor1=sqrt((x123-sensor1_x).^2+(y123-sensor1_y).^2);
d_sensor2=sqrt((x123-sensor2_x).^2+(y123-sensor2_y).^2);
d_sensor3=sqrt((x123-sensor3_x).^2+(y123-sensor3_y).^2);

c=3.0e+8;   %光速   
%送信機からセンサーまでの実際の到着時間
t1=d_sensor1./c; t2=d_sensor2./c; t3=d_sensor3./c;

x=zeros(1,len); y=zeros(1,len);
%偶数列を0にします
if rem(len,2)==0  %lenが偶数の場合
    for i=1:len/2  
%測位   
    j=2*i-1;
    [x(1,j),y(1,j)]=MYTOA(t1(1,j),t2(1,j),t3(1,j),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
    end
%2つの円の交点を求める
L=sqrt((sensor2_x-sensor1_x).^2+(sensor2_y-sensor1_y).^2);
sita=atan((sensor2_y-sensor1_y)./(sensor2_x-sensor1_x));
alpha=acos((L.^2+d_sensor1.^2-d_sensor2.^2)./(2*L.*d_sensor1));
x_1=sensor1_x+d_sensor1.*cos(sita+alpha); y_1=sensor1_y+d_sensor1.*sin(sita+alpha);
x_2=sensor1_x+d_sensor1.*cos(sita-alpha); y_2=sensor1_y+d_sensor1.*sin(sita-alpha);
%それぞれの交点と前回の測位点との距離
d_prepoint1=sqrt((x(1,1:2:len)-x_1(1,2:2:len)).^2+(y(1,1:2:len)-y_1(1,2:2:len)).^2);
d_prepoint2=sqrt((x(1,1:2:len)-x_2(1,2:2:len)).^2+(y(1,1:2:len)-y_2(1,2:2:len)).^2);
%距離が近いほうの交点を採用
I=find((d_prepoint1-d_prepoint2)<0);
J=find((d_prepoint1-d_prepoint2)>0);
x(1,2*I)=x_1(1,2*I); y(1,2*I)=y_1(1,2*I);
x(1,2*J)=x_2(1,2*J); y(1,2*J)=y_2(1,2*J);

else     %lenが奇数の場合
    for i=1:(len+1)/2  
%測位   
    j=2*i-1;
    [x(1,j),y(1,j)]=MYTOA(t1(1,j),t2(1,j),t3(1,j),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
    end
%2つの円の交点を求める
L=sqrt((sensor2_x-sensor1_x).^2+(sensor2_y-sensor1_y).^2);
sita=atan((sensor2_y-sensor1_y)./(sensor2_x-sensor1_x));
alpha=acos((L.^2+d_sensor1.^2-d_sensor2.^2)./(2*L.*d_sensor1));
x_1=sensor1_x+d_sensor1.*cos(sita+alpha); y_1=sensor1_y+d_sensor1.*sin(sita+alpha);
x_2=sensor1_x+d_sensor1.*cos(sita-alpha); y_2=sensor1_y+d_sensor1.*sin(sita-alpha);
%それぞれの交点と前回の測位点との距離
d_prepoint1=sqrt((x(1,1:2:len-1)-x_1(1,2:2:len)).^2+(y(1,1:2:len-1)-y_1(1,2:2:len)).^2);
d_prepoint2=sqrt((x(1,1:2:len-1)-x_2(1,2:2:len)).^2+(y(1,1:2:len-1)-y_2(1,2:2:len)).^2);
%距離が近いほうの交点を採用
I=find((d_prepoint1-d_prepoint2)<0);
J=find((d_prepoint1-d_prepoint2)>0);
x(1,2*I)=x_1(1,2*I); y(1,2*I)=y_1(1,2*I);
x(1,2*J)=x_2(1,2*J); y(1,2*J)=y_2(1,2*J);
end

figure(1) %表示
hold on
%　測位場所とセンサーの配置表示
p1=plot(x(1,1:2:len),y(1,1:2:len),'bo');   %３つのセンサーを使用
p2=plot(x(1,2:2:len),y(1,2:2:len),'ro');    %２つのセンサーを使用
%plot(x_1,y_1,'co'); plot(x_2,y_2,'bo');
plot(sensor1_x,sensor1_y,'ks'); 
plot(sensor2_x,sensor2_y,'ks'); plot(sensor3_x,sensor3_y,'ks');
%　歩行ルートの表示
plot(x123,y123,'g-');
%　パルス発生位置の表示
plot(x123,y123,'k+');
%枠
rectangle('Position',[1 1 31.1534 25]); axis([-1 33.1534 -1 27]);
hold off
legend([p1 p2],{'3sensor','2sensor'}); %凡例