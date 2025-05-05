function [fitresult, gof] = createFit(levels_cut, l_s_axis_x)
[xData, yData] = prepareCurveData( levels_cut, l_s_axis_x );
ft = fittype( 'poly2' );
[fitresult, gof] = fit( xData, yData, ft, 'Normalize', 'on' );


