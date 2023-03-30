

worldARUCOSize = 0.08;
worldARUCONoise = 0.002;



lm = [8,8];
% Apply realistic noise to a landmark measurement
% Compute projected square sides
squareprojL = (lm(2) - worldARUCOSize/2) / lm(1);
squareprojR = (lm(2) + worldARUCOSize/2) / lm(1);
% Add some noise
squareprojL = squareprojL + randn()*worldARUCONoise;
squareprojR = squareprojR + randn()*worldARUCONoise;
% Compute the landmark from this
lmproj = (squareprojL + squareprojR)/2;
lm1 = worldARUCOSize / (squareprojR - squareprojL);
% Make sure depth is in range
if lm1 < 0 || lm1 > 10
    lm1 = 10;
end

noisy_lm = [lm1 ; lmproj * lm1]