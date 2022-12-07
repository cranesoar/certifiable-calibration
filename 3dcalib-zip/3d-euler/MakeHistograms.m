%
% Copyright © 2012, The Massachusetts Institute of Technology. All rights reserved. 
%
% THE LICENSOR EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES CONCERNING THIS 
% SOFIWARE AND DOCUMENTATION, INCLUDING ANY WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR ANY PARTICULAR PURPOSE, NON- INFRINGEMENT AND WARRANTIES OF 
% PERFORMANCE, AND ANY WARRANTY THAT MIGHT OTHERWISE ARISE FROM COURSE OF 
% DEALING OR USAGE OF TRADE. NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH 
% RESPECT TO THE USE OF THE SOFIWARE OR DOCUMENTATION. Under no circumstances 
% shall the Licensor be liable for incidental, special, indirect, direct or 
% consequential damages, or loss of profits, interruption of business, or 
% related expenses which may arise from use of Software or Documentation, 
% including but not limited to those resulting from defects in Software 
% and/or Documentation, or loss or inaccuracy of data of any kind. 
%
% This software is licensed under the "LIMITED RESEARCH LICENSE (SOURCE
% CODE)" as described in the included LICENSE.txt
%
% Please cite the paper below if you are using this software in your work:
% Brookshire, J.; Teller, S. Extrinsic Calibration from Per-Sensor Egomotion. 
%   Robotics: Science and Systems, 2012.
%
function MakeHistograms(data, plot_titles, num_bins, rows, cols, varargin)

    num_bins = ceil(num_bins);
    
    show_variances = 0;
    vert_lines = [];
    vert_markers = [];
    show_fit = 1;
    show_mean = 0;
    barcolor = [173 235 255]/255;
    includeMeanInTitle = 1;
    includeStdInTitle = 1;
    for i = 1:size(data,2)
        additional_text{i} = '';
    end
    for i = 1:length(varargin)
        if strcmpi(varargin{i}, 'showvar')
            show_variances = 1;
        elseif strcmpi(varargin{i}, 'VertLines')
            vert_lines = varargin{i+1};
        elseif strcmpi(varargin{i}, 'VertMarkers')
            vert_markers = varargin{i+1};
        elseif strcmpi(varargin{i}, 'ShowFit')
            show_fit = varargin{i+1};
        elseif strcmpi(varargin{i}, 'showmean')
            show_mean = 1;
        elseif strcmpi(varargin{i}, 'titletexts')
            additional_text = varargin{i+1};
        elseif strcmpi(varargin{i}, 'BarColor')
            barcolor = varargin{i+1};
        elseif strcmpi(varargin{i}, 'IncludeMeanInTitle')
            includeMeanInTitle = varargin{i+1};
        elseif strcmpi(varargin{i}, 'IncludeStdInTitle')
            includeStdInTitle = varargin{i+1};
        end
    end
    
    for i = 1:size(data,2)
        
        tight_subplot(rows, cols, i);
        %subplot(rows,cols,i);
        d = data(:,i);
        hist(d, num_bins);
        h = findobj(gca,'Type','patch');
        set(h,'FaceColor',barcolor,'EdgeColor','w')
        [n,x]=hist(d, num_bins);
        hold on;
        m=mean(d);
        s=std(d);
        
        ax=axis;
        %q=max(abs(ax(1:2)-m));
        q = 3*s;
        axis([m-q m+q ax(3:4)])
        ax = axis;
        line(ax(1:2), [0 0], 'Color', 'k')
        
        ax = axis;
        if show_fit == 1
            %x0 = min(x):range(x)/100:max(x);
            x0 = linspace(ax(1),ax(2));
            y0 = 1/sqrt(2*pi*s^2)*exp(-(x0-m).^2/(2*s^2));
            plot(x0, y0/max(y0)*max(n),'r');
        end
        if show_mean==1
            %y = [ax(3) (ax(4)-ax(3))/2+ax(3)];
            %plot([m m], y, '-k', 'LineWidth', 1);
            plot([m m], ax(3:4), '-k', 'LineWidth', 3);
        end
        if show_variances==1
            %y = [ax(3) (ax(4)-ax(3))/2+ax(3)];
            %plot([m-s m-s], y, '--k', [m+s m+s], y, '--k', 'LineWidth', 1);
            plot([m-s m-s], ax(3:4), '--k', [m+s m+s], ax(3:4), '--k', 'LineWidth', 3);
        end
        if size(vert_lines, 2)>=1
            %y = [(ax(4)-ax(3))/2+ax(3) ax(4)];
            %plot([vert_lines(i,1) vert_lines(i,1)], y, '-g', 'LineWidth', 1);
            plot([vert_lines(i,1) vert_lines(i,1)], ax(3:4), '-g', 'LineWidth', 2);
        end
        for j = 2:size(vert_lines, 2)
            %y = [(ax(4)-ax(3))/2+ax(3) ax(4)];
            %plot([vert_lines(i,j) vert_lines(i,j)], y, '--g', 'LineWidth', 1);
            plot([vert_lines(i,j) vert_lines(i,j)], ax(3:4), '--g', 'LineWidth', 2);
        end
        if size(vert_markers, 2)>=1
            plot([vert_markers(i,1) vert_markers(i,1)], ax(3)*[1 1], 'g^', 'LineWidth', 2, 'MarkerSize',8);
        end
        for j = 2:size(vert_markers, 2)
            plot([vert_markers(i,j) vert_markers(i,j)], ax(3)*[1 1], 'gd', 'LineWidth', 2, 'MarkerSize',8);
        end
        hold off;
        
        title_text = plot_titles{i};
        if includeMeanInTitle==1
            title_text = [title_text sprintf(', \\mu_E=%0.4f',m)];
        end
        if includeStdInTitle==1
            title_text = [title_text sprintf(', \\sigma_E=%0.4f', s)];
        end
        title_text = [title_text additional_text{i}];
        title(title_text);
       
    end