% matrix

left = imread('left.png');
right = imread('right.png');

%grayscale
left = rgb2gray(left);
right = rgb2gray(right);



%Edge
coeffs = fspecial('log',15, 0.16 ); 
left = imfilter(left,coeffs,'replicate');
right = imfilter(right,coeffs,'replicate');

figure( 10 )
subplot(1,2,1)
imshow(left)
subplot(1,2,2)
imshow(right)

% figure(2)
% cyan=stereoAnaglyph(left,right);
% imshow(cyan);
% title('Red-cyan composite view of the stereo images');

%double (otherwise error)
%left = im2double(left);
%right = im2double(right);

figure( 11)
subplot(1,2,1)
imshow(left)
subplot(1,2,2)
imshow(right)

m=size(right,1);
n=size(right,2);
N=int32(n/7);
diff=zeros(m,n,N);
left_slid=zeros(m,n);
right_slid=zeros(m,n);
for i=1:N
   left_slid(:,1:n-i+1)= left(:,i:n);
%    right_slid(:,1:n-i+1)= right(:,i:n);
 diff(:,:,i) = abs(right - left_slid);
%  diff(:,:,i) = left - right_slid ;

%Block Filter
 diff(:,:,i) = imfilter( diff(:,:,i),ones([17,17]),'replicate');

 

%  figure(1)
%  imshow(diff(:,:,i), [0,20]);
end

% Find minimum
[~,loc]=min(diff,[],3);

figure(4)
subplot(1,2,1)
title('Disparity Map');
imshow(loc,[0,100])
colormap jet
colorbar


d=disparity(left,right,'BlockSize',17,'DisparityRange',[0 64]);
subplot(1,2,2)
imshow(d, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

K = wiener2(loc,[2 2]);
figure(5), imshow(K)
