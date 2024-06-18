function h = multiplot(x, y1, y2, x_label, y_label,y1_legend, y2_legend, pdf_name)

    set(0, 'DefaultTextInterpreter', 'latex')
    set(0, 'DefaultLegendInterpreter', 'latex')
    set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
    
    lw = 2;
    %h = figure();
    h = figure('Renderer', 'painters', 'Position', [10 10 900 350]);
    removeToolbarExplorationButtons(h)

    plot(x, y1, 'Linewidth', lw);
    xlim([x(1) x(end)])
    hold on
    plot(x, y2, 'Linewidth', lw,'LineStyle','--');
    
    xlabel(x_label)
    ylabel(y_label)
    set(gcf,'color','w');
    legend(y1_legend, y2_legend);

    set(gca, 'FontSize',16);
    grid on
    box on
    %exportgraphics(h, pdf_name);
    set(0, 'DefaultTextInterpreter', 'none')
    set(0, 'DefaultLegendInterpreter', 'none')
    set(0, 'DefaultAxesTickLabelInterpreter', 'none')

end

