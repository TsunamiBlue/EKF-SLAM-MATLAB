ax = gca();
color = repmat([255,0,0],560,1);
scatter(ax, xes,yes);
freezeColors(ax);
hold(ax, 'on');
imh = imshow(ax, 'tra.fig');
hold(ax, 'off')
uistack(imh, 'bottom')