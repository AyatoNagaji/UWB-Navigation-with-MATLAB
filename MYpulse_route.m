function[x11,y11,x22,y22,x33,y33,x123,y123,len]=MYpulse_route()

step=1/2.7; %[m]　１秒に３回のパルス発生間隔で時速４キロ移動

a1=8/5; b1=0;           %歩行コースのパラメータ
a2=-2/3; b2=17/3;
a3=35/30; b3=-21.5/3;

delta_x1=step/sqrt(1+a1^2);  %step[m]間隔になるように刻み幅を決定
delta_x2=step/sqrt(1+a2^2);  
delta_x3=step/sqrt(1+a3^2);

%各x座標の決定　ここから
max_x1=12*delta_x1;
x1=0:delta_x1:max_x1;  x11=0:0.01:2.5; %パルス用と歩行ルート用
y1=a1*x1+b1;        y11=a1*x11+b1; 
norm1=norm([max(x1) a1*max(x1)+b1]-[2.5 4]); 
syms min_x2
eqn1=norm([min_x2 a2*min_x2+b2]-[2.5 4])+norm1==step;
min_x2=solve(eqn1,min_x2);
row1=find(min_x2(:,1)>=2.5);
first_x2=min_x2(row1,1);

max_x2=14*delta_x2+first_x2;
x2=first_x2:delta_x2:max_x2;  x22 =2.5:0.01:7;
y2=a2*x2+b2;        y22=a2*x22+b2;
norm2=norm([max(x2) a2*max(x2)+b2]-[7 1]); 
syms min_x3
eqn2=norm([min_x3 a3*min_x3+b3]-[7 1])+norm2==step;
min_x3=solve(eqn2,min_x3);
row2=find(min_x3(:,1)>=7);
first_x3=min_x3(row2,1);

max_x3=12*delta_x3+first_x3;
x3=first_x3:delta_x3:max_x3;   x33 =7:0.01:10;
y3=a3*x3+b3;        y33=a3*x33+b3;
%　ここまで

len=length(x1)+length(x2)+length(x3);
x123=zeros(1,len); y123=zeros(1,len); 
x123=[x1 x2 x3]; 
a=zeros(1,len); b=zeros(1,len);
a(1,1:13)=a1; a(1,14:28)=a2; a(1,29:41)=a3;
b(1,1:13)=b1; b(1,14:28)=b2; b(1,29:41)=b3;
%パルス発生位置の式
y123=a.*x123+b;

%figure(1)
%hold on
%plot(x11,y11,'g-'); plot(x22,y22,'g-'); plot(x33,y33,'g-');
%%　パルス発生位置の表示
%plot(x123,y123,'b+');
%%枠
%rectangle('Position',[0 0 10 5]); axis([-1 11 -1 6]); xlabel('[m]') 
%ylabel('[m]')
%hold off