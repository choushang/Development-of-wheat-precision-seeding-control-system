import numpy as np

# 状态转移模型
def state_transition(x, u, dt):
    v_x, v_y, theta = x
    a_x, a_y, omega = u
    v_x_new = v_x + a_x * dt
    v_y_new = v_y + a_y * dt
    theta_new = theta + omega * dt
    return np.array([v_x_new, v_y_new, theta_new])

# 观测函数
def observation_function(x):
    v_x, v_y, theta = x
    speed = np.sqrt(v_x ** 2 + v_y ** 2)  # 速度估计
    return np.array([speed, speed, theta])

# 扩展卡尔曼滤波函数，修改为支持分开的acc_x和acc_y
def extended_kalman_filter(Gnss_speed, Radar_Speed, acc_x, acc_y, angleY, dt, Q, R):
    # 初始状态：速度估计 (gnss_speed, radar_speed), 初始角度 (angleY)
    x = np.array([Gnss_speed, Radar_Speed, angleY])  # 初始状态
    P = np.eye(3)  # 初始协方差矩阵

    # 控制输入：加速度和角速度
    u = np.array([acc_x, acc_y, 0])  # 控制输入（加速度和角速度），这里只使用加速度（假设角速度为0）

    # 预测步骤
    x_pred = state_transition(x, u, dt)
    F = np.eye(3)  # 线性化的状态转移矩阵
    P_pred = F @ P @ F.T + Q

    # 更新步骤
    H = np.array([[x_pred[0] / np.sqrt(x_pred[0] ** 2 + x_pred[1] ** 2), 0, 0],
                  [x_pred[1] / np.sqrt(x_pred[0] ** 2 + x_pred[1] ** 2), 0, 0],
                  [0, 0, 1]])  # 观测矩阵
    z = np.array([Gnss_speed, Radar_Speed, angleY])  # 测量值
    y = z - observation_function(x_pred)  # 残差
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)  # 卡尔曼增益
    x = x_pred + K @ y
    P = (np.eye(3) - K @ H) @ P_pred

    # 融合后的速度（x和y方向速度的模）
    estimated_speed = np.linalg.norm(x[:2])

    # 如果速度不合理，回退到GNSS速度
    if abs(estimated_speed - Gnss_speed) > 0.5:
        estimated_speed = Gnss_speed

    return estimated_speed

# 将EKF过程封装成一个可调用的函数，支持分开的acc_x和acc_y
def get_fused_speed(Gnss_speed, Radar_Speed, acc_x, acc_y, angleY, dt):
    # Q和R的最优值（假设已通过调优得出）
    Q_opt = np.diag([0.01690025, 0.01690025, 0.01690025])
    R_opt = np.diag([0.00386079942806028, 0.00386079942806028, 0.00386079942806028])

    # 调用扩展卡尔曼滤波器进行速度融合
    fused_speed = extended_kalman_filter(
        Gnss_speed=Gnss_speed,
        Radar_Speed=Radar_Speed,
        acc_x=acc_x,
        acc_y=acc_y,
        angleY=angleY,
        dt=dt,
        Q=Q_opt,
        R=R_opt
    )

    return fused_speed
