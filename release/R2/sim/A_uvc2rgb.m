%%
clc
fp = fopen('raw.txt','r');
a = fscanf(fp,'%x');
fclose(fp);


%%
clc
X = 1024;
Y = 768;
I = zeros(Y,X,3);
d2 = zeros(Y*X*3,1);
toread = Y*X*3;

%%
clc
count = 1;
i = 1;
ct = '';
cc = '';

img = 0;

if img == 0 
    fid = '80';
    nfid = '81';
    eofid = '82'; % '82' or '83'
    data = a;
else
    fid = '81';
    nfid = '80';
    eofid = '83'; % '82' or '83'
    data = a(25177:end);
end



while (strcmp(ct,'C0')==0) && (strcmp(cc,nfid)==0)

ct = dec2hex(data(i),2);
cc = dec2hex(data(i+1),2);
display([ct ' ' cc])

if strcmp(ct,'0C')==1 && strcmp(cc,fid)==1
    d2(count:count+499) = data(i+12:i+511); 
    count = count + 500;
    i = i + 512;
elseif strcmp(ct,'0C')==1 && strcmp(cc,eofid)==1
    d2(count:count+(toread-count)) = data(i+12:i+12+toread-count); 
    i = i + 12 +(toread-count)+1;
    count = count + (toread-count);
   
end 

end

i

%%

count = 1;

for yy = 1:Y
for xx=1:X
    I(yy,xx,1) = d2(count); count = count+1;
    I(yy,xx,2) = d2(count); count = count+1;
    I(yy,xx,3) = d2(count); count = count+1;    
end 
end

I2 = (mat2gray(I));
imshow(I2)
