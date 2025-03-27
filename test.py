from pyntcloud import PyntCloud

# 读取 PLY 文件
cloud = PyntCloud.from_file("cloud_04_room05.ply")

# 获取点云数据
points = cloud.points

# 打印点云数据
print(points)
