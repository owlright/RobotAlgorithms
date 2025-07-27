set terminal qt font "PingFang SC,12"
set view 60, 30              # 设置观察角度
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set title "3D Point Cloud Visualization"

set grid xtics ytics ztics # Adding grid for better visualization
set tics
set border

# 设置坐标轴样式
set xyplane at 0

set zeroaxis lt -1 lw 2

a = 1
b = 2
c = 3
d = 4

splot (-a*x - b*y - d)/c with lines title "true 平面", \
    "plane_points.txt" using 1:2:3 with points pointtype 7 pointsize 1 lc rgb "blue" title "生成随机点云", \
    "fitted_plane_points.txt" using 1:2:3 with points pointtype 7 pointsize 1 lc rgb "red" title "拟合平面点云"


pause -1 "Press any key to continue..."