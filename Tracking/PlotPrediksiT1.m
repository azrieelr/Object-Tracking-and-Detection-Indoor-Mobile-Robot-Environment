clc
for j = 1:TrackNum
    figure(j);clf 
%     figtitle = sgtitle('some initial title','FontSize',12);
    subplot(211)
    plot(Logs{j}.xV_enkf(1,1:end),'b');
    hold on
    plot(Logs{j}.xV_enkf(1,2:end),'r');
    xlabel('time-step')
    ylabel('X(position)')
    grid on 
    legend('x_k','x_{k+1}')
    subplot(212)
    plot(Logs{j}.xV_enkf(2,1:end),'b');
    hold on
    plot(Logs{j}.xV_enkf(2,2:end),'r');
    xlabel('time-step')
    ylabel('Y(position)')
    grid on 
    legend('y_k','y_{k+1}')
%     newfigtitle = ['Comparison Prediction Position t+1 for Object ',num2str(j)];
%     figtitle.String = newfigtitle;
end
