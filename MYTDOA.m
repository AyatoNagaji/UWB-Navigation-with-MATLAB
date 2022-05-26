%function=MYTDOA(t12,t13,t14)

C=3e+8;
x1=1; y1=0.5;
x2=1; y2=4.5;
x3=9; y3=4.5;
x4=9; y4=0.5;


t12=3e-9; t13=1e-9; t14=1e-9;

A=2*[x1-x2 y1-y2 C*t12; x1-x3 y1-y3 C*t13; x1-x4 y1-y4 C*t14];
B=[x1^2-x2^2+y1^2-y2^2+C^2*t12^2; x1^2-x3^2+y1^2-y3^2+C^2*t13^2;...
    x1^2-x4^2+y1^2-y4^2+C^2*t14^2];
Answer=A\B;
x = Answer(1);
y = Answer(2);

figure(1)
hold on
plot(x,y,'rx');
plot(x1,y1,'o');
plot(x2,y2,'o');
plot(x3,y3,'o');
plot(x4,y4,'o');
%axis([0 10 0 5]);
grid on;

