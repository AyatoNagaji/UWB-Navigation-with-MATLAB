function[x,y]=MYTOA_2sensor(t1,t2,sensor1_x,sensor1_y,sensor2_x,sensor2_y,t3)

[x11,y11,x22,y22,x33,y33,x123,y123,len]=MYpulse_route(); 

d_sensor1=zeros(1,len); 
d_sensor2=zeros(1,len);
x=zeros(1,len); y=zeros(1,len);
c=3.0e+8;   %光速
%センサーと送信機の距離を求める
d_sensor1=sqrt((x123-sensor1_x).^2+(y123-sensor1_y).^2);
d_sensor2=sqrt((x123-sensor2_x).^2+(y123-sensor2_y).^2);

%偶数列を0にします
if rem(len,2)==0  %lenが偶数の場合
    for i=1:len/2  
%測位   
    j=2*i-1;
    [x(1,j),y(1,j)]=MYTOA(t1(1,j),t2(1,j),t3(1,j));
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
    [x(1,j),y(1,j)]=MYTOA(t1(1,j),t2(1,j),t3(1,j));
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

%x(:,2:2:len)=zeros(size(x(1,2:2:len))); 偶数列を０にできる