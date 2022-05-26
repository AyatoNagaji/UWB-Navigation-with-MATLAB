function[x,y]=MYTOA(t1,t2,t3,x1,y1,x2,y2,x3,y3)

%x1=6; x2=(sqrt(28^2-10^2)+3)/2+3; x3=(sqrt(28^2-10^2)+3)/2+3;
%y1=13; y2=18; y3=8;

c=3.0e+8;
%l1=6.7; l2=7; l3=7;
%t1=l1/c; 
%t2=l2/c; 
%t3=l3/c;

A=2*[x1-x2 y1-y2; x1-x3 y1-y3];
B=([c^2*(t2^2-t1^2)+x1^2-x2^2+y1^2-y2^2;...
    c^2*(t3^2-t1^2)+x1^2-x3^2+y1^2-y3^2]);
Answer=vpa(A\B);
x = Answer(1);
y = Answer(2);




%t=linspace(0,2*pi,100);

%figure(1)
%hold on
%plot(x,y,'k+');
%plot(l1*sin(t)+x1,l1*cos(t)+y1);
%plot(l2*sin(t)+x2,l2*cos(t)+y2);
%plot(l3*sin(t)+x3,l3*cos(t)+y3);
%%　測位場所とセンサーの配置表示 
%plot(x1,y1,'bs'); plot(x2,y2,'bs'); plot(x3,y3,'bs');
%rectangle('Position',[1 1 33.1534 25]); axis([-1 35.1534 -1 27]);
%label('[m]') 
%ylabel('[m]')
%hold off