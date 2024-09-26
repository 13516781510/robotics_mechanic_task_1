from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH,SerialLink,ctraj,jtraj,mtraj
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# 定义每个关节的参数
L1 = RevoluteDH(d=0.762, a=0, alpha=-np.pi/2, offset=np.deg2rad(0))
L2 = RevoluteDH(d=0.393412, a=0, alpha=-np.pi/2, offset=np.deg2rad(0))
L3 = PrismaticDH(theta=0, a=0, alpha=0, offset=0)
L4 = RevoluteDH(d=0.2268, a=0, alpha=-np.pi/2, offset=np.deg2rad(0))
L5 = RevoluteDH(d=0, a=0, alpha=-np.pi/2, offset=np.deg2rad(0))
L6 = RevoluteDH(d=0.4318, a=0, alpha=0, offset=np.deg2rad(0))


# 指定关节角度
joint_angles0 = [0, 0, 0, 0, 0, 0]
joint_angles1 = [0, -np.deg2rad(90), 0.635, 0, np.deg2rad(180), np.deg2rad(180)]
joint_angles2 = [np.deg2rad(20), -np.deg2rad(110), 0.660, np.deg2rad(20), np.deg2rad(170), np.deg2rad(170)]
joint_angles3 = [np.deg2rad(40), -np.deg2rad(130), 0.700, np.deg2rad(40), np.deg2rad(160), np.deg2rad(160)]
joint_angles4 = [np.deg2rad(60), -np.deg2rad(150), 0.735, np.deg2rad(60), np.deg2rad(150), np.deg2rad(150)]


# 定义机械臂
stanford = DHRobot([L1, L2, L3, L4, L5, L6], name='Stanford Arm')
# 指定关节角度

traj1 = jtraj(joint_angles0,joint_angles1,100)

traj2 = jtraj(joint_angles1,joint_angles2,100)

traj3 = jtraj(joint_angles2,joint_angles3,100)

traj4 = jtraj(joint_angles3,joint_angles4,100)
# 设置暂停时间（秒）
pause_time = 0.01  # 可以调整这个值使动画变慢

# 创建一个函数用于逐帧显示轨迹
id = 0
def animate_trajectory(robot, traj, pause_time):

        global id
        robot.plot(traj.q,limits=[-0.2,1.5,-0.2,1.4,-1,2],backend='pyplot',movie = f"movie_{id}.mp4",block=True)
        id = id+1
# 显示轨迹
animate_trajectory(stanford, traj1, pause_time)
animate_trajectory(stanford, traj2, pause_time)
animate_trajectory(stanford, traj3, pause_time)
animate_trajectory(stanford, traj4, pause_time)
