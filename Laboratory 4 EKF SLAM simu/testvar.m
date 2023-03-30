trail_axes = gca();
xlim(trail_axes, [-5 5]);
ylim(trail_axes, [-5 5]);
axis(trail_axes, 'manual');
state_ids = EKF.state_ids;
for i = 1:numel(state_ids)
    color = state_ids(i);
    scatter(state_vector(3+2*i-1),state_vector(3+2*i),8,'MarkerFaceColor'...
        ,cmap(color),'MarkerEdgeColor','none','parent',trail_axes);
    
    text(state_vector(3+2*i-1)+0.1, state_vector(3+2*i)+0.1, cellstr(num2str(color))...
        , 'Color',cmap(color), 'FontSize', 12, 'Parent', trail_axes);
    
%         [e1,e2] = elpse_cal(state_vector(3+2*i-1:3+2*i),cov(3+2*i-1:3+2*i,3+2*i-1:3+2*i));
%         plot(e1,e2,'Parent',trail_axes,'Color',cmap(color));

    e0 = plot_ellipses(state_vector(3+2*i-1:3+2*i),cov(3+2*i-1:3+2*i,3+2*i-1:3+2*i)...
        ,cmap(color),trail_axes);
    
end