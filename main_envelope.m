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

%チャネル応答・白色ガウス雑音を経て、誤差が生じた到着時間を求める
t1=MYtimeerror2(3,10,t1_real,len);
t2=MYtimeerror2(3,10,t2_real,len);
t3=MYtimeerror2(3,10,t3_real,len);

x=zeros(1,len); y=zeros(1,len);
%測位
parfor i=1:len
    [x(1,i),y(1,i)]=MYTOA(t1(1,i),t2(1,i),t3(1,i),sensor1_x,sensor1_y,sensor2_x,sensor2_y,sensor3_x,sensor3_y);
end

%  ウィンドウ内の最小値を1次関数で補間して下からの包絡線を求める
%　ウィンドウサイズkにおける誤差を求める。
error=zeros(1,100); % 純粋な測位誤差
error2=zeros(1,100); % リアルタイムにおける最大測位誤差
for k=1:100
%k=8;
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
    min_check=max([I1(1,1) I2(1,1) I3(1,1)]); 
    max_check=min([I1(1,length(I1)) I2(1,length(I2)) I3(1,length(I3))]);
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
    error(1,k)=d_error_min_ave; %ウィンドウサイズkごとの平均誤差
    error2(1,k)=error(1,k)+(2*k-2)/100*5/3.6; %ウィンドウサイズkごとの平均誤差

    if k==8  % k=８付近でリアルタイムにおける最大測位誤差は最小となることがこの研究で分かったため、その時のグラフを表示 
        figure(1)
        hold on
        %　測位結果
        plot(x(1,min_check:max_check),y(1,min_check:max_check),'r.');
        plot(x123,y123,'k-','LineWidth',2);
        plot(x_min(1,min_check:max_check),y_min(1,min_check:max_check),'b.');
        %  センサーの配置表示
        plot(sensor1_x,sensor1_y,'ks'); plot(sensor2_x,sensor2_y,'ks'); 
        plot(sensor3_x,sensor3_y,'ks');
        %  ラベル
        xlabel('[m]','Fontsize',14); ylabel('[m]','Fontsize',14);
        title('測位結果')
        legend('何も工夫をしない従来法','提案手法','真値（移動ルート）','センサー')
        hold off

        figure(2)
        hold on
        plot(d_sensor1(1,1:len),'m','LineWidth',2);
        plot(t1(1,1:len).*c);
        plot(I1(1,1):I1(1,length(I1)),t1_min(1,I1(1,1):I1(1,length(I1))).*c,'b');
        legend('真値','測距データ','下からの包絡線')
        xlabel('測距番号','Fontsize',15); ylabel('測距値[m]','Fontsize',15);
        title('送信局と受信局の距離（時系列の測距データ）')
        hold off
    end
   
end

%最小の誤差とその時のウィンドウサイズkを記録
[G,min_k]=min(error);   
[G2,min_k2]=min(error2);


figure(3)
hold on
% 最小値を☆でプロット
plot(error); 
plot(error2);
plot(min_k,G,'mp');
plot(min_k2,G2,'rp');
xlabel('ウィンドウサイズkの値','Fontsize',16); ylabel('誤差[m]','Fontsize',16);
legend('純粋な即位誤差','リアルタイムにおける最大測位誤差')
hold off
