function smartplot(rr,cc,offset,x,signals,titles)
    figure;
    L = length(titles);
    for jj = 1 : L
        sig=signals{jj};
        for ii = 1 : size(sig,1)
            subplot(rr,cc,ii+offset(jj));plot(x,sig(ii,:));
            title([titles{jj} num2str(ii)]); grid on;
        end
    end
    set(gcf,'Units','Normalized','OuterPosition',[0 0 1 1]);
    drawnow;
end