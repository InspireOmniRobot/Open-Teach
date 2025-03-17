import numpy as np


def print_matrix(name, matrix):
    """漂亮地打印矩阵"""
    print(f"\n{name}:")
    for row in matrix:
        print("[", end=" ")
        for elem in row:
            print(f"{elem:5.2f}", end=" ")
        print("]")


final = np.array(
    [
        [0, 0, 1],  # 将第2个分量变为第1个
        [1, 0, 0],  # 将第3个分量变为第2个
        [0, 1, 0],  # 将第1个分量变为第3个
    ]
)

# 3. 构建齐次变换矩阵
H_R_V = np.zeros((4, 4))
H_R_V[:3, :3] = final
H_R_V[3, 3] = 1

print_matrix("最终齐次变换矩阵", H_R_V)

# 4. 验证结果
# 测试一个向量 [1, 0, 0] 的变换
test_vector = np.array([1, 2, 3])
transformed = final @ test_vector
print("\n测试向量 [1, 2, 3] 变换后:", transformed)
