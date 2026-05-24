function realCsvFitDisableAxesInteractions(ax)
try
    ax.Interactions = [];
catch
end
try
    ax.Title.Interactions = [];
catch
end
try
    ax.XLabel.Interactions = [];
catch
end
try
    ax.YLabel.Interactions = [];
catch
end
try
    ax.ZLabel.Interactions = [];
catch
end
try
    ax.UIContextMenu = [];
catch
end
end
