function [cc,cr,cc2,cr2,w,h,flag]=extractball(Imwork,Imback,index)
cc = 0;
cr = 0;
cc2=0;
cr2=0;
w=0;
h=0;
flag = 0;
[MR,MC,Dim] = size(Imwork);%�����ĳߴ�

% subtract��ȥ background & select pixels���ص� with a big difference
fore = zeros(MR,MC);          %image subtracktion
% fore = (abs(Imwork(:,:,1) - Imback(:,:,1)) > 85)& (abs(Imwork(:,:,2) - Imback(:,:,2)) < 100)&(abs(Imwork(:,:,3) - Imback(:,:,3)) < 100);
% fore = (abs(Imwork(:,:,1)) <90)& (abs(Imwork(:,:,2)) > 100)&(abs(Imwork(:,:,3)) > 100);
% fore=(((Imwork(:,:,2))/(Imwork(:,:,2)+Imwork(:,:,1)+Imwork(:,:,3)))>0.3)&(abs(Imwork(:,:,2)) > 110);
fore=(abs(Imwork(:,:,2))-abs(Imwork(:,:,1)) >38) & (abs(Imwork(:,:,1) < 170));
%  & abs(Imwork(:,:,3)) -abs(Imwork(:,:,1))<25
%      figure (2)
%    imshow(fore)
% Morphology Operation  erode to remove small noise
foremm = bwmorph(fore,'erode',1); %��ʴ��������С������bwmorph����: ��ȡ������ͼ�������. �﷨��BW2 = bwmorph(BW1,operation,n) �� nΪ������operation=��erode�������ýṹԪ��ones��3������ʴ���㣻
foremm = bwmorph(foremm,'dilate',3);
% foremm = bwmorph(foremm,'erode',1);
% foremm = bwmorph(foremm,'dilate',1);
% foremm = bwmorph(foremm,'erode',1);
% foremm = bwmorph(foremm,'dilate',1);
foremm = bwmorph(foremm,'erode',1);
foremm = bwmorph(foremm,'fill',3);
foremm = bwmorph(foremm,'erode',1);


% imshow(foremm)
% select largest object
labeled = bwlabel(foremm,8);%bwlabel�ڶ�ֵͼ���б����ͨ������ L = bwlabel(BW,n) ����һ����BW��С��ͬ��L����n��ֵΪ4��8����ʾ�ǰ�4��ͨѰ�����򣬻���8��ͨѰ�ң�Ĭ��Ϊ8��
stats = regionprops(labeled,['basic']);

[N,W] = size(stats);%�ܹ���N��Ŀ����ͼ����
if N < 1%���û��Ŀ��ͷ���
    return
end

% ð�����򣨴Ӵ�С���Ա���Ƿ�ֻ��һ��Ŀ��
id = zeros(N);
for i = 1 : N
    id(i) = i;
end
for i = 1 : N-1
%      if Im(stats(1).Centroid,1)>50
   for j = i+1 : N
%        if Im(stats(j).Centroid,1)>50
        if stats(i).Area < stats(j).Area %��stasts����Ŀ�꣩��������Ӵ�С��˳������
            tmp = stats(i);
            stats(i) = stats(j);
            stats(j) = tmp;
            tmp = id(i);
            id(i) = id(j);
            id(j) = tmp;%ͬ��id�еı��Ϊ�����С������ı��
         end
%        end
   end
%     end
end

% ȷ��ͼƬ��������һ���������
if stats(1).Area < 40 %��֤������һ���㹻�������
    return %����������������С��100���򷵻�
end
% if N > 15 %��֤������һ���㹻�������
%     return %����������������С��100���򷵻�
% end
if N==1
selected = (labeled==id(1));%selectedΪ0��1����1��Ӧ��id(1)��Ŀ�꣬���������Ŀ��
centroid = stats(1).Centroid;%���Ŀ�������
w=stats(1).BoundingBox(3);         %��ȡ���εĳ��Ϳ�
h=stats(1).BoundingBox(4);
cc = centroid(1);%
cr = centroid(2);
cc2=0;
cr2=0;
flag = 1;
end
if (N~=1) && (stats(1).Area > 80) && (stats(2).Area < 80) 
selected = (labeled==id(1));%selectedΪ0��1����1��Ӧ��id(1)��Ŀ�꣬���������Ŀ��
centroid = stats(1).Centroid;%���Ŀ�������
w=stats(1).BoundingBox(3);         %��ȡ���εĳ��Ϳ�
h=stats(1).BoundingBox(4);
cc = centroid(1);%
cr = centroid(2);
cc2=0;
cr2=0;
flag = 1;
end
if (N~=1) && (stats(1).Area > 80) && (stats(2).Area > 80)
selected = (labeled==id(1));%selectedΪ0��1����1��Ӧ��id(1)��Ŀ�꣬���������Ŀ��
centroid = stats(1).Centroid;%���Ŀ�������
centroid2 = stats(2).Centroid;%���Ŀ�������
w=stats(1).BoundingBox(3);         %��ȡ���εĳ��Ϳ�
h=stats(1).BoundingBox(4);
cc = centroid(1);%
cr = centroid(2);
cc2=centroid2(1);
cr2=centroid2(2);
flag = 1;
end
return