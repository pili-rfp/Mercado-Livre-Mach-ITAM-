\ENCODING=ISO-8859-1
\Problem name: mlibre

Maximize
 obj1: 4 x_0 + 2 x_1 + 3 x_2 + 5 x_3 + x_4
Subject To
 LB_limit:              4 x_0 + 2 x_1 + 3 x_2 + 5 x_3 + x_4 >= 5
 UB_limit:              4 x_0 + 2 x_1 + 3 x_2 + 5 x_3 + x_4 <= 12
 lb_aisles_restriction: y_0 + y_1 + y_2 + y_3 + y_4 >= 1
 ub_aisles_restriction: y_0 + y_1 + y_2 + y_3 + y_4 <= 5
 availability_0:        3 x_0 + x_3 - 2 y_0 - 2 y_1 - 2 y_3 <= 0
 availability_1:        x_1 + x_4 - y_0 - y_1 - 2 y_2 - y_3 - y_4 <= 0
 availability_2:        x_0 + x_2 + 2 x_3 - y_0 - 2 y_1 - 2 y_4 <= 0
 availability_3:        x_1 + x_3 - y_2 - y_3 - y_4 <= 0
 availability_4:        2 x_2 + x_3 - y_0 - y_1 - 2 y_2 - y_3 - 2 y_4 <= 0
Bounds
 0 <= x_0 <= 1
 0 <= x_1 <= 1
 0 <= x_2 <= 1
 0 <= x_3 <= 1
 0 <= x_4 <= 1
 0 <= y_0 <= 1
 0 <= y_1 <= 1
 0 <= y_2 <= 1
 0 <= y_3 <= 1
 0 <= y_4 <= 1
Binaries
 x_0  x_1  x_2  x_3  x_4  y_0  y_1  y_2  y_3  y_4 
End
