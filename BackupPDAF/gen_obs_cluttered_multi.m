function DataList = gen_obs_cluttered_multi(TrackNum, PoseXObs, PoseYObs,PoseORObs)
%function DataList = gen_obs_cluttered_multi(TrackNum, PoseXObs, PoseYObs, PoseORObs)
    % Noise std
    r = 0.25;
    
    % For simplicity clutter has been normally distributed around each of
    % the targets.
    clutter_rate = 5; % Number of clutter measurements per target
    r_clutter = 1.5;
    
    DataList = [];
    for i=1:size(PoseXObs,1)

        for j=1:TrackNum
            %if (PoseXObs(i,j)~=0 && PoseYObs(i,j)~=0)
            if (PoseXObs(i,j)~=0 && PoseYObs(i,j)~=0 && PoseORObs(i,j)~=0)
                %DataList(:,j,i) = [PoseXObs(i,j)+normrnd(0,r^2), PoseYObs(i,j)+normrnd(0,r^2)];
                DataList(:,j,i) = [ PoseXObs(i,j)+normrnd(0,r^2), PoseYObs(i,j)+normrnd(0,r^2), PoseORObs(i,j)+normrnd(0,r^2)];
            end
        end
        
        % Clutter for target 1 (always present)
        for j=TrackNum+1:TrackNum+clutter_rate
             DataList(:,j,i) = [PoseXObs(i,1)+normrnd(0,r_clutter^2), PoseYObs(i,1)+normrnd(0,r_clutter^2)];
             %DataList(:,j,i) = [ PoseXObs(i,1)+normrnd(0,r_clutter^2), PoseYObs(i,1)+normrnd(0,r_clutter^2), PoseORObs(i,1)+normrnd(0,r_clutter^2)];
        end
        
        % Clutter for target 2 
        if (TrackNum>=2)
            for j=TrackNum+clutter_rate+1:TrackNum+2*clutter_rate
                 %DataList(:,j,i) = [ PoseXObs(i,2)+normrnd(0,r_clutter^2), PoseYObs(i,2)+normrnd(0,r_clutter^2)];
                 DataList(:,j,i) = [ PoseXObs(i,2)+normrnd(0,r_clutter^2), PoseYObs(i,2)+normrnd(0,r_clutter^2), PoseORObs(i,2)+normrnd(0,r_clutter^2)];
            end
        end
        
        % Clutter for target 3
        if (TrackNum>=3)
            for j=TrackNum+2*clutter_rate+1:TrackNum+3*clutter_rate
                 %DataList(:,j,i) = [PoseXObs(i,3)+normrnd(0,r_clutter^2), PoseYObs(i,3)+normrnd(0,r_clutter^2)];
                 DataList(:,j,i) = [ PoseXObs(i,3)+normrnd(0,r_clutter^2), PoseYObs(i,3)+normrnd(0,r_clutter^2), PoseORObs(i,3)+normrnd(0,r_clutter^2)];
            end
        end
    end
end
