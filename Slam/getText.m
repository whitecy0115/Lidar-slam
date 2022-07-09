function text = getText(cam,detector)

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
        
    else
        text = 'No';
    end
end