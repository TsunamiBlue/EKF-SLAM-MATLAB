function [k0,ki,ci] = compute_gain(ebar,dt,y,num_lm)
% using distance.
    numObserve = num_lm(1,y(4,:));
    k0 = 0.15;
    ki = zeros(size(y,2)); %number of detected landmarks currently
    ci = zeros(size(y,2)); 
    timeImpact = 1/dt; % get rid of any time slots. to be time-invariant.
    distance = zeros(1,size(y,2));
    

    % measure the err distance of each observed point
    for i = 1:size(ebar,2)
        distance(i) = norm((ebar(1:2,i)-[0;0]));
    end
    
    
    if ~isempty(y)
       for i = 1:size(y,2)
          distance_gain = (1-(distance(i))/max(distance,[],2));
          if max(distance,[],2)==0
              distance_gain = 1;
          end
          % ki gains for landmark observer counts
          ki(i,i) = timeImpact * (1.5/(numObserve(i))^(1/2) *1);    
          %ki(i,i) = 0;
          if norm(ebar(1:2,i))==0
              ci(i,i) = 1;
          else
              %ci(i,i) = 0.08 * norm(ebar(1:2,i));
              ci(i,i) = 0.8;
              %ci(i,i) = 1/size(y,2);
          end
       end
    end
    
end