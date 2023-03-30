function rmse = RMSE(posex,landmarks)
% ------inputs------
% posex: observed landmarks positions in time t
% landmarks: ground truth lm w.r.t observed landmarks
    n = size(posex,2);
    p = posex(1:2,:);
    
    rmse = sqrt(sum((p-landmarks).^2,'all')/n);
end