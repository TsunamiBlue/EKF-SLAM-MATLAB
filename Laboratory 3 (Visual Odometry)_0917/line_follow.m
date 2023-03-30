function [u,q,eol_flag] = line_follow(img, v)
    
    default_speed = v;
    eol_flag = false;
    % Binarise the image by picking some sensible threshold
    gry_img = rgb2gray(img);
    bin_img = ~imbinarize(gry_img, 0.2);
    bin_img = bin_img(end-49:end-20, :);

    % Check the close line is visible
    if ~any(bin_img)
        u = nan;
        q = nan;
        eol_flag = true;
        return;
    end

    % find the close line coordinates in the image
    [rows, cols] = find(bin_img);
    
    %Normalizing and Find the centre of the line to follow
    line_centre = (mean(cols)-400/2)/200;
  
    % If you have reached the end of the line, you need to stop by breaking
    % the loop.
    end_of_line = false;
    if (end_of_line)
        eol_flag = true;
    end

    % Use the line centre to compute a velocity command
    u = 0.1*default_speed*(1-line_centre^2);
    q = -2*line_centre;

end