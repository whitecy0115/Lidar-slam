new = imread('test_img.png');
size(new)
imshow(new);impixelinfo;
%% 
I = imread('image_2.tif');
figure; imshow(I);impixelinfo;


%% 
load("Adress_Detector.mat");
load("occGrid.mat");
%% 
tic;

count =0;

figure;
while count<=50
    [bboxes,scores] = detect(detector,I);

if isempty(bboxes) == 0
    label{1} = num2str(scores);
    label{2} = 'center';
    bbox = [bboxes;320 240 10 10];
    img = insertObjectAnnotation(I,'rectangle',bbox,label);
    imshow(img);drawnow;
    count = count+1;
end
toc
end
%% 
I = imread("image_716.tif");

%% 


    [bboxes,~] = detect(detector,I);

    im = im2gray(I);
    im = imadjust(im);
    Icorrected = imtophat(im,strel('disk',15));
    img_bi = imbinarize(Icorrected);
    
    roi =bboxes;
    ocrResults = ocr(img_bi, roi);
    img_bi = double(img_bi);
    label = '1201';
    img = insertObjectAnnotation(img_bi,'rectangle',bboxes,label);
  

figure;imshow(img);

%% 
% show(planner);impixelinfo;
start = [6.25	7.4	0.7]; % pi: left 0: right pi/2: up -pi/2: down
goal = [ -2.8    11.3  pi];

[path_re,planner]= HybridAStar(occGrid,start,goal);
show(planner)
