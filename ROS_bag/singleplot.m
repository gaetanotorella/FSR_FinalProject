function h = singleplot(y, x_label, y_label, title_str, legend_vec)
    
    lw = 2;

    plot(y, 'Linewidth', lw);
    
    xlabel(x_label)
    ylabel(y_label)

    set(gcf,'color','w');
    set(gca, 'FontSize',16);
    grid on
    box on
    legend(legend_vec)
    title(title_str)

end

