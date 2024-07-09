function h = plot_ts(y, x_label, y_label, title_str, legend_vec)
    
    lw = 2;
    
    x = y.Time;
    plot(y, 'Linewidth', lw);
    xlim([x(1) x(end)])
    xlim([371.0520 423.9380])

  
    xlabel(x_label)
    ylabel(y_label)

    set(gcf,'color','w');
    set(gca, 'FontSize',12);
    grid on
    box on
    legend(legend_vec)
    title(title_str)


end

