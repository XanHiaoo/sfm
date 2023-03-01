#  opencv实现sfm

## 基于SfM与CMVS+PMVS的单目视觉三维重建https://zhuanlan.zhihu.com/p/385281495

![image-20211208190233797](sfm_opencv.assets/image-20211208190233797.png)

##  一.sift特征点提取

```python
sift = cv2.AKAZE_create()
```

```python
sift.detectAndCompute()
```

AkAZE是KAZE的加速版 与SIFT,SUFR比较: 1.更加稳定 2.非线性尺度空间 3.AKAZE速度更加快 4.比较新的算法,只有Opencv新的版本才可以用

获得连续图片的

>keypoints_for_all, descriptors_for_all, colors_for_all

##  二.连续间图片特征点匹配(使用k近邻检查排除异常点)

 ###  只需相邻图片进行匹配 1-2 2-3 ...N-1 - N 一共N-1对匹配

- ```
  bf =  cv2.NORM_HAMMING2
  ```

- ```
  flann = cv2.FlannBasedMatcher(index_params,search_params)
  ```

  用FLANNE算法对两张照片进行关键点匹配，返回这两张照片的匹配对
  
  Fast Library for Approximate Nearest Neighbors. 它包含一组算法，这些算法针对大型数据集中的快速最近邻搜索和高维特征进行了优化.对于大型数据集，它比BFMatcher工作得更快.
  
- 这里用了BFMatcher

-  随后做bf.knnMatch ,使用knnmatch进行最近邻匹配

  > knnMatch返回K个好的匹配，k可以自行指定。这里指定k=2，raw_matches = matcher.knnMatch(desc1, desc2,2) ，然后每个match得到两个最接近的descriptor，再计算最接近距离和次接近距离之间的比值，当比值大于某个设定的值时，才作为最终的match。
  >
  > ```
  > # Knnmatch与match的返回值类型一样，只不过一组返回的俩个DMatch类型：
  > # 返回值是：这俩个DMatch数据类型是俩个与原图像特征点最接近的俩个特征点（match返回的是最匹配的）只有这俩个特征点的欧式距离小于一定值的时候才会认为匹配成功。
  > ```

@TODO

- 图片的顺序可能影响sfm稀疏点云生成,尝试逆序生成后增量添加

## 三.将第一张图片与第二张图片进行初始重构

- 需要参数

  >```
  >K(内参矩阵),
  >                                                                                   keypoints_for_all,
  >                                                                                   colors_for_all,
  >                                                                                   matches_for_all
  >                                                                                                                                                               
  >```

- 找到前两幅图像匹配对坐标

  >```
  >out_p1 = np.asarray([p1[m.queryIdx].pt for m in matches])  # queryIndex 查询图像中描述符的索引
  >out_p2 = np.asarray([p2[m.trainIdx].pt for m in matches])
  >```

-  计算外参

- ```
  重建
  structure, proj = reconstruction(K, R0, T0, R, T, p1, p2)
  ```

## 四.增量方式添加剩余点云...

- 获取第i幅图中匹配点的空间三维坐标，以及第i+1幅图匹配点的像素坐标

- 重复第一步步骤，生成连续图像点云

- 点云融合（fusion_structure）

   - ```
     如果新生成的三维点已存在，则告诉图i+1是哪个空间点
     ```

   - ```
     如果新生成的三维点不存在于原有点云中，则加入新点
     ```

## 五.删除误差较大的点

@TODO 离群点去除

## 六.BA优化









代码疑问：

```
cv2.triangulatePoints
```





一些idea：

1. 图片的顺序可能影响sfm稀疏点云生成

![image-20211208165101392](sfm_opencv.assets/image-20211208165101392.png)

![image-20211208165134254](sfm_opencv.assets/image-20211208165134254.png)