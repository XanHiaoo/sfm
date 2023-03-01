#!/usr/bin/ python
# -*- coding: utf-8 -*-
# @Time : 2021/12/8 
# @Author : XIAO
# @File : test.py
# import open3d as o3d
# print("Load a ply point cloud, print it, and render it")
# pcd = o3d.io.read_point_cloud("./reconstruction/dense/pmvs/models/option-0000.ply")
# o3d.visualization.draw_geometries([pcd])
import numpy as np
# a=[0,0,1,1,0,1]
# a=np.array(a)
# b=[1,2,3,4,5,6]
# b=np.array(b)
# c=b[a!=0]
# print(c)
import numpy as np
a=np.array([0,0,1])
b=np.array([0.12,0.91,0.39])
cosangle = round(a.dot(b)/(np.linalg.norm(a) * np.linalg.norm(b)),2)
print(cosangle)
angle = np.arccos(cosangle)
sinangle=round(np.sin(angle),2)
print(sinangle)
mtx=np.array([[1,0,0],
     [0,cosangle,sinangle],
     [0,-sinangle,cosangle]])
print(mtx)