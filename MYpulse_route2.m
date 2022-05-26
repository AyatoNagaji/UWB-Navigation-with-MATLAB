function[x123,y123,len,step]=MYpulse_route2()

step=(1/10)*5/3.6; %[m]　1秒に10回のパルス発生間隔で時速5キロ移動

%歩行コースのパラメータ
x1_start=0;  y1_start=0; 
x1_last=25; y1_last=y1_start;

x2_start=x1_last; y2_start=y1_last;
x2_last=x2_start; y2_last=y2_start+25;

x3_start=x2_last; y3_start=y2_start+25;
x3_last=x3_start-25; y3_last=y3_start;

x4_start=x3_last; y4_start=y3_start;
x4_last=x4_start; y4_last=y3_start-25;
%ここまで


x1=x1_start:step:x1_last;
x1_len=length(x1);
x1_extra=abs(x1_last-x1_start)-(x1_len-1)*step;
y1=ones(1,x1_len)*y1_start;

y2=y2_start+sqrt(step^2-x1_extra^2):step:y2_last;
y2_len=length(y2);
y2_extra=abs(y2_last-y2_start-sqrt(step^2-x1_extra^2))-(y2_len-1)*step;
x2=ones(1,y2_len)*x2_start;

x3=x3_start-sqrt(step^2-y2_extra^2):-step:x3_last;
x3_len=length(x3);
x3_extra=abs(x3_last-x3_start+sqrt(step^2-y2_extra^2))-(x3_len-1)*step;
y3=ones(1,x3_len)*y3_start;

y4=y4_start-sqrt(step^2-x3_extra^2):-step:y4_last;
y4_len=length(y4);
y4_extra=abs(y4_last-y4_start)-(y4_len-1)*step;
x4=ones(1,y4_len)*x4_start;


len=x1_len+y2_len+x3_len+y4_len;
x123=zeros(1,len); y123=zeros(1,len); 
x123=[x1 x2 x3 x4];
y123=[y1 y2 y3 y4];

%a=sqrt((x123-[0 x123(1,1:len-1)]).^2+(y123-[0 y123(1,1:len-1)]).^2);

%figure(1) 
%hold on
%o=linspace(0,2*pi,100);
%plot(28*sin(o)+25,28*cos(o)+0);
%plot(28*sin(o),28*cos(o));
%plot(28*sin(o),28*cos(o)+25);
%plot(28*sin(o)+25,28*cos(o)+25);
%　歩行ルートの表示
%plot(x123,y123,'g-');
%　パルス発生位置の表示
%plot(x123,y123,'bx');
%rectangle('Position',[ -3 -3 31 31]);
%xlabel('x [m]','Fontsize',16); ylabel('y [m]','Fontsize',16);
%plot(sensor1_x,sensor1_y,'ks'); plot(sensor2_x,sensor2_y,'ks'); 
%plot(sensor3_x,sensor3_y,'ks'); 
%xlim([-3 28]); ylim([-3 28]);
%hold off
%figure(2)
%plot(a);