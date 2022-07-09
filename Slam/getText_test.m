function [text,img] = getText_test(cam,detector)

    I = snapshot(cam);
    [bboxes,~] = detect(detector,I);

    im = im2gray(I);
    im = imadjust(im);
    Icorrected = imtophat(im,strel('disk',15));
    img_bi = imbinarize(Icorrected);
    
    roi =bboxes;
    ocrResults = ocr(img_bi, roi);

    if isempty(ocrResults) == 0
        text = ocrResults.Text;
        label = '1202';
        img = insertObjectAnnotation(img_bi,'rectangle',bboxes,label);
    else
        text = 'No';
    end
end