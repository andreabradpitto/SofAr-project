
n = NJOINTS;qmin = QMIN; qmax = QMAX; qdotmax = QDOTMAX; B = QMARGIN;
ETA = 5;
SAT = evalin('base','sat','0');

%if (rotrefw), smartplot(3,3,[0 3 6 9],t,{xg,ag,wg},{'xg','ag','wg'}); end
smartplot(3,3,[0 3 6],t,{eta(:,:),rho(:,:),etadot(:,:)},{['ETA = ',num2str(ETA),',SAT=',num2str(sat),',eta'],...
    ['ETA = ',num2str(ETA),',SAT=',num2str(sat),',rho'],['ETA = ',num2str(ETA),',SAT=',num2str(sat),',etadot']});
if evalin('base','plotq')
    figure,set(gcf,'Units','Normalized','OuterPosition',[0 0 1 1]);
    for ii = 1 : n
        subplot(3,3,ii);plot(t,q(ii,1:length(t)));title(['ETA = ',num2str(ETA),....
            ',SAT=',num2str(sat),',q' num2str(ii)]); grid on;
        hold on;
        plot(t,ones(1,tn)*qmin(ii),'color','red');
        plot(t,ones(1,tn)*qmax(ii),'color','red');
        plot(t,ones(1,tn)*qmin(ii)+B,'color',[ 0.9100 0.4100 0.1700]);
        plot(t,ones(1,tn)*qmax(ii)-B,'color',[ 0.9100 0.4100 0.1700]);
        drawnow;
    end
end

if evalin('base','plotqdot',false)
    figure,set(gcf,'Units','Normalized','OuterPosition',[0 0 1 1]);
    for ii = 1 : n
        subplot(3,3,ii);plot(t,qdot(ii,1:length(t)));grid on;hold on;
        plot(t,-ones(1,tn)*qdotmax(ii),'color','red');
        plot(t,ones(1,tn)*qdotmax(ii),'color','red');
        title(['ETA = ' num2str(ETA),',SAT=',num2str(sat),',qdot' num2str(ii)]);
        set(gcf,'Units','Normalized','OuterPosition',[0 0 1 1]);
        drawnow;
    end
end

if evalin('base','plotcurves',false)
    figure,set(gcf,'Units','Normalized','OuterPosition',[0 0 1 1]);
    tn = size(xg,2);
    xe = reshape(TOe(1,4,:),[1,tn]);
    ye = reshape(TOe(2,4,:),[1,tn]);
    ze = reshape(TOe(3,4,:),[1,tn]);
    plot_both_curves(xg(1,:),xg(2,:),xg(3,:),xe,ye,ze);
end

function plot_both_curves(xorg,yorg,zorg,x,y,z)
    tn = size(xorg,2);
    grid on;
    plot3(xorg,yorg,zorg,'Color','red');hold on;
    plot3(x,y,z,'Color','blue');
    text(x(1),y(1),z(1),'ee start');
    text(x(tn),y(tn),z(tn),'ee end');
    text(xorg(1),yorg(1),zorg(1),'trgt start');
    text(xorg(tn),yorg(tn),zorg(tn),'trgt end');
    legend('target','ee');
end