# 悬挂测试场高度图

`heightmap.pgm` 是内置地形的 8 位量化导出：65 × 65、P5 二进制 PGM，黑色 0 米、白色 1.2 米。世界范围 x/z 各 -12 到 12 米，左上为 (-12,-12)。渲染几何和轮地接触共同使用加载的高度。

在仓库根目录运行：

```sh
./build/project_Demo/vehicle_playground/demo_vehicle_playground --heightmap "$PWD/Resource/vehicle_playground/heightmap.pgm"
```

默认不传地图时使用程序生成的浮点高度，和该图的高度差最多约 2.4 毫米。
