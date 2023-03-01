#!/usr/bin/ python
# -*- coding: utf-8 -*-
# @Time : 2021/12/26 
# @Author : XIAO
# @File : model_extract.py
import config
import open3d as o3d
import numpy as np
def Plane_Segmentatio(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.15,
                                             ransac_n=1000,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud,round(a,2),round(b,2),round(c,2)

def Point_cloud_outlier_removal(pcd,nb_neighbors, std_ratio):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    downpcd_inlier_cloud = pcd.select_by_index(ind)
    return downpcd_inlier_cloud

def Point_cloud_centralization(pcd):
    '''
    点云坐标中心化
    :param pcd:
    :return: pcd
    '''
    xyz = np.asarray(pcd.points)
    xyz = xyz - np.mean(xyz, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd

def Point_cloud_rotation(pcd,x,y,z):
    a = np.array([0, 0, 1])
    b = np.array([x,y,z])
    cosangle = round(a.dot(b) / (np.linalg.norm(a) * np.linalg.norm(b)), 2)
    print(cosangle)
    angle = np.arccos(cosangle)
    sinangle = round(np.sin(angle), 2)
    print(sinangle)
    xyz = np.asarray(pcd.points)
    mtx = np.array([[1, 0, 0,0],
                    [0, cosangle, sinangle,0],
                    [0, -sinangle, cosangle,0],
                   [0, 0, xyz[:,2].min(), 1]])
    mtx1 = np.array([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 1, 1]])

    xyz = np.insert(xyz, 3, values=1, axis=1)
    xyz=np.dot(xyz, mtx)
    xyz = np.dot(xyz, mtx1)
    print(xyz)

    xyz= np.delete(xyz, 3,axis = 1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    return pcd

#ply文件路径
model_path=config.re_dir+'model/model.ply'
modelextract_path=config.re_dir+'model/model_extract.ply'
modelextract_path1=config.re_dir+'model/model_extract_centralization.ply'
modelextract_path2=config.re_dir+'model/model_extract_rotation.ply'
#读入
pcd=source=o3d.io.read_point_cloud(model_path)
o3d.visualization.draw_geometries([pcd])

#分离最大平面（地面）
inner,outer,a,b,c=Plane_Segmentatio(source)
o3d.visualization.draw_geometries([outer])

#离群点去除
outer=Point_cloud_outlier_removal(outer,1000,2)
o3d.visualization.draw_geometries([outer])
outer_org=outer

#点云坐标中心化
outer=Point_cloud_centralization(outer)

#点云坐标旋转,平移
outer=Point_cloud_rotation(outer,a,b,c)
o3d.visualization.draw_geometries([outer])

#点云保存
o3d.io.write_point_cloud(modelextract_path2,outer)

#表面(mesh)重建（球体旋转）
'''
球枢轴算法 (BPA) [Bernardini1999]是一种与 alpha 形状相关的表面重建方法。直观地，想象一下我们落在点云上的具有给定半径的 3D 球。如果它击中任何 3 个点（并且它没有穿过这 3 个点），它会创建一个三角形。然后，算法从现有三角形的边缘开始旋转，每次它击中球没有穿过的 3 个点时，我们创建另一个三角形。
'''
radii = [0.005, 0.01, 0.02, 0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    outer_org, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([rec_mesh])
