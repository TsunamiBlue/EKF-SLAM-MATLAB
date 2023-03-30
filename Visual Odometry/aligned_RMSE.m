function armse = aligned_RMSE(pose0,landmarks)
% ------inputs------
% pose0: currently observed landmarks positions
% landmarks: all ground truth positions of lms corresponds to observed landmarks
    n = size(pose0,2);
    p = pose0(1:2,:);
    
    % compute average of pose and ground truth, span in dim = 2
    uhat =  mean(p,2);
    u = mean(landmarks,2);
    Sigma = ((landmarks-u)*(p-uhat)')/n;
    [U,D,V] = svd(Sigma);
    
    if det(Sigma)>=0
        A = [1 0;0 1];
    else
        A = [1 0;0 -1];
    end
    
    Rs = V*A*U';
    xs = uhat - Rs*u;
    armse = sqrt(sum((Rs'*(p-xs) - landmarks).^2,'all')/n);

end