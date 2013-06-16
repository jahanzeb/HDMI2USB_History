clc
fp = fopen('image_raw.txt','r');
a = fscanf(fp,'%x');
fclose(fp);

aa = diff(a);
sum(aa(:))
hist(aa)

%%
Y = 8;
X = 1024;
I = zeros(Y,X,3);

b = dec2hex(a,6);

r = b(:,1:2);
g = b(:,4:3);
b = b(:,6:5);

r = reshape(r,Y,X);
g = reshape(g,Y,X);
b = reshape(b,Y,X);

I(:,:,1) = r;
I(:,:,2) = g;
I(:,:,3) = b;


