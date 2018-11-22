%delete(obj);
function [u,v]=KK(error1,error2,pe, obj)
f=810.83424;
% global obj
% delete(instrfind({'Port'},{'COM6'}));

u=640;
v=320;

% global port;
% port=serial('COM6');
% set(port,'BaudRate',9600);
% set(port,'DataBits',8);
% set(port,'StopBits',1);
% fopen(port);



%
% N=5;                                           %��ʼ����������Ҫ��֡��
%  mov = VideoReader('sample.avi');                %��ȡAVI��Ƶ�ļ�
%  vidFrames=read(mov);
%  frameQTY = get(mov, 'numberOfFrames');          %��ȡ��Ƶ֡��
% tmFr = vidFrames(:, :, :, 1);         %��ɻҶ�ͼ
% [MR,MC,Dim] = size(tmFr);

% ���㱳��ͼƬ
Imzero = zeros(480,640,3);  % ������ʼ��, ����240��320��3ȫ���������

% ��ʼ��Kalman�˲���
% R=[[0.2845,0.0045]',[0.0045,0.0455]'];      %�۲�Э�������
% H=[[1,0]',[0,1]',[0,0]',[0,0]'];            %��������
% Q=0.01*eye(4);                              %ϵͳЭ�������, eye���ɵ�λ���� s=eye(n) ����n*n��λ����
% P = 100*eye(4);                             %��һʱ�̵�Ԥ�����Э�������
% dt=10;
% A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];  %��ʱ��k��k+1��״̬xx(k-1,:)��ԾǨ����
% g = 1;                                              % pixels^2/time step
% Bu = [0,0,0,g]';           %����˹����������ǰһ֡Ԥ�⵱ǰ֡��Ԥ��ֵʱ��BuӰ��Ԥ��ֵ�ķ���.
% kfinit=0;
x=zeros(10000,4);

Im1=[];Im2=[];Im3=[];Im4=[];Im5=[];Imwork1=[];Imback1=[];
% ѭ������ͼƬloop over all video
M=[];
Im1=[];
for i = 2
    tic
    % load image
    %     Im =getsnapshot(obj);
    Im =snapshot(obj);
    %     Im5 =getsnapshot(obj);
    %     Im5= ycbcr2rgb(Im5);
    %     if x(i-1,1)~=0
    %         if x(i-1,1)-200>0
    %             KKK1=x(i-1,1)-200;
    %         else
    %             KKK1=0;
    %         end
    %       if x(i-1,2)-200>0
    %             KKK2=x(i-1,2)-200;
    %         else
    %             KKK2=0;
    %       end
    %
    %       if x(i-1,2)+200>640
    %             KKK3=640;
    %         else
    %             KKK3=x(i-1,2)+200;
    %       end
    %       if x(i-1,1)+200>480
    %             KKK4=480;
    %         else
    %             KKK4=x(i-1,1)+200;
    %         end
    %
    %
    %     Im1=Im(KKK1:KKK4,KKK2:KKK3,1:3);
    %     Im= ycbcr2rgb(Im1);
    % %     Im2(:,:,:,i)=Im1;
    %     end
    %
    %  if x(i-1,1)==0
    %      Im= ycbcr2rgb(Im);
    %     Im2(:,:,:,i)=Im;
    %  end
%     figure (2)
    imshow(Im);
    hold on
    %     title_time = sprintf('t = %6.4f', t);
%     title_tip  = sprintf('Camera Position: x:%6.4f, y:%6.4f, z:%6.4f', ...
%         pe(1), pe(2), pe(3));
%     title_error = sprintf('error: %0.4f [pixel]', ...
%         sqrt(error1^2+error2^2));
    
    
    %     text(10,20,title_tip,'Color','white','FontSize',12);
    %     text(10,40,title_error,'Color','white','FontSize',12);
    
%     title({
%         %             title_time
%         ['\fontsize{24}',title_tip];
%         title_error
%         });
    %     hold off
    %      figure (2)
    %      imshow(Im(:,:,2));
    %      figure (3)
    %      imshow(Im(:,:,3));
    %     if(i <= N)
    %         Imzero = Imzero + double(Im);
    %     else
    %         %��ʼ�˶���⣬���ȼ��������
    %         if(i == N+1)
    %             Imback = Imzero/N;
    %         else
    %             f1 = double(Im2(:, :, :, i-5));
    %             Imzero = Imzero + double(Im) - f1;   %��������
    %             Imback = Imzero/N;
    %         end
    Imback=[];
    Imback1=[];
    Imwork = double(Im);
    %extract ball
    %         if x(i-1,1)~=0
    %         [cc(i),cr(i),w,h,flag]=extractball(Imwork,Imback,i);
    % %           if cc(i)==0
    % %         [cc(i),cr(i),w,h,flag]=extractball(Imwork,Imback,i)
    %         cc(i)=KKK1+cc(i);% x
    %         cr(i)=KKK2+cr(i);% y
    % %           end
    %         else
    [cc(i),cr(i),cc2(i),cr2(i),w,h,flag]=extractball(Imwork,Imback,i);
    %         end
    %         figure(3)
    plot(cc(i),cr(i),'r.','MarkerSize',25); %  �������(cc(i),cr(i))���̵��ʾ
%     plot(cc2(i),cr2(i),'r.','MarkerSize',25);
             hold on
    if flag==0
        continue
    end
    
    % ���ʵ�ʹ켣
    %[cc(i),cr(i)]
    %�ӵ�(cc(i)-radius,cr(i)-radius)��ʼ����һ�����Σ����Ϊ2*radius ����Ϊ2*radius.������(cc(i),cr(i)),
    %         rectangle('Position',[cc(i)-w/2,cr(i)-h/2,w,h], 'edgecolor','green');
    % ����kalman�˲���
    %         if kfinit==0
    %             xp = [MC/2,MR/2,0,0]'  ;     %��ʼ��Ԥ��״̬
    %         else
    %             xp=A*x(i-1,:)' + Bu    ;     %Ԥ��δ��״̬
    %         end
    %
    %         kfinit=1;
    %         PP = A*P*A' + Q  ;   %Ԥ�����Э�������
    %         K = PP*H'*inv(H*PP*H'+R) ; %����, Inv ��������
    %         x(i,:) = (xp + K*([cc(i),cr(i)]' - H*xp))';  %����״̬,�����º��״̬�� x(n,:); ��ȡ����ĵ�n�У�x(:,n); ��ȡ����ĵ�n��
    %         P = (eye(4)-K*H)*PP  ; %�������,�����º��Э����
    %
    %         % kalmanԤ�������
    %          hold on
    %          plot(x(i,1),x(i,2),'r.') %Ԥ����˶�Ŀ������(x(i,1),x(i,2))�Ժ���ʾ
    %          rectangle('Position',[x(i,1)-w/2,x(i,2)-h/2,w+50,h+50], 'edgecolor','red');
    
    x_round=floor(cc(i));
    y_round=floor(cr(i));
    
    if  cc2(i)~=0 && cr2(i)~=0
        cc1(i)=(cc(i)+cc2(i))/2;
        cr1(i)=(cr(i)+cr2(i))/2;
        plot(cc1(i),cr1(i),'b.','MarkerSize',40)
        %            hold on
        x_round=floor(cc1(i));
        y_round=floor(cr1(i));
        
    end
    u=x_round;
    v=y_round;
    %  �������(cc(i),cr(i))���̵��ʾ
    %         uu=[x_round,y_round]
    %        [q1,q2]=T(uu)
    %        q1*180/3.14
    %        q2*180/3.14\
%     pause(0.01);
end

% end
%%  �ر�
end

