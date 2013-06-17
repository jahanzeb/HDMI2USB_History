I = imread('img.bmp');
imagesc(I)
fp = fopen('rgb.hex','w');

[Y X Z] = size(I)

for y=1:Y
    for x=1:X
        fprintf(fp,'%02X%02X%02X\n',I(y,x,1),I(y,x,2),I(y,x,3));
    end 
end

fclose(fp);
clc