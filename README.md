#概述

此文件夹中是SZTU-1034战队在2026RobotCon比赛中梅林区的路径生成器。
根据战队使用的路径规划算法，利用线性插值和梯形速度规划的路径生成器，可以根据输入的两个点位自动生成最短路径。
利用Cline + glm-5.1，使用C++编写

#文件

-path_generator.cpp：主程序添加main宏定义方便调试
-path_generator.h：头文件支持移植其他工程中
-generated_path_three_segments.txt：路径生成的txt文档包含三段路径数组方便调试
-AIREADME.md：为了方便代码修改与维护生成该markdown文件让AI快速理解代码
-README.me：用户使用说明
