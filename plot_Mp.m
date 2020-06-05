function plot_Mp(betas,Mps)
    hold on
    
    L=length(betas);
    plot(betas,Mps);
    plot(betas,ones(L,1)*1.06,'k--');
    plot(betas,ones(L,1)*1.13,'k--');
    hold off
end