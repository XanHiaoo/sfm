#!/usr/bin/ python
# -*- coding: utf-8 -*-
# @Time : 2021/12/7 
# @Author : XIAO
# @File : config.py
import numpy as np

Imgdir='./image/bicycle_a6000' #重建图片文件夹
Intrinsicsdir='./Intrinsics/sonya6000'#相机内参文件夹
MRT = 0.7
K = np.load(Intrinsicsdir+'/mtx.npy')#相机内参矩阵
distort_coefs=np.load(Intrinsicsdir+'/dist.npy')#相机矫正矩阵
x = 0.5
y = 1
re_dir=Imgdir+'/'#sfm重建存储位置
