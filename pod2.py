# pod2.py
import Hexapod as pd
import threading
import ActionGroupControl as AGC
import time

PI = pd.PI



'''
函数说明：
pd.set_body_pos(x, y, z)：设置下个动作机身位置

pd.set_velocity(Vx, Vy, omega)：设置机器人行走方向和速度
速度为矢量，因此拆分为Vx,Vy。如果要自转，Vx和Vy一定要为0。

使用pd.set_body_pos(x, y, z)或者pd.set_velocity(Vx, Vy, omega)后
一定要接一句pd.move(step)。不然不会运动。
'''
# 休眠1秒
time.sleep(1)

# 初始化六足机器人
pd.hexapod_init()

'''
key_map = {"PSB_CROSS": 0, "PSB_CIRCLE": 1, "PSB_SQUARE": 3, "PSB_TRIANGLE": 4,
           "PSB_L1": 6, "PSB_R1": 7, "PSB_L2": 8, "PSB_R2": 9,
           "PSB_SELECT": 10, "PSB_START": 11}
'''
# 按钮按下时执行的函数
'''
具体编写：把pass删掉，替换为pd.set_body_pos(x, y, z)，
pd.set_velocity(Vx, Vy, omega)即可
'''
def set_PSB_SELECT_button(): #SELECT按钮
    pass
 
def set_PSB_R1_button(): #R1按钮
    pd.set_body_pos(0, 0, 100)

def set_PSB_R2_button(): #R2按钮
    pd.set_body_pos(0, 0, 20)
    
def set_PSB_L1_button(): #L1按钮
#     th = threading.Thread(target=AGC.runActionGroup, args=('7.7taijie', 0), daemon=True)  # 运行动作函数是阻塞式的，如果要循环运行一段时间后停止，请用线程来开启
#     th.start()
#     time.sleep(10)
#     AGC.stopAction()  # 3秒后发出停止指令
#     while th.is_alive(): # 等待动作完全停止
#         time.sleep(0.01)
# 
# #     AGC.runActionGroup('7.7taijie')  # 参数为动作组的名称，不包含后缀，以字符形式传入
#
    pass
def set_PSB_L2_button(): #L2按钮
    pd.set_velocity(0, 0, 0)

def set_PSB_SQUARE_button(): #正方形按钮
    pd.MAX_R_PACE = 60
    pd.set_velocity(0, 0, 200)

def set_PSB_CIRCLE_button(): #圆按钮
    pd.MAX_R_PACE = 60
    pd.set_velocity(0, 0, -200)

def set_PSB_TRIANGLE_button(): #三角形按钮
    pd.set_step_mode()

def set_PSB_CROSS_button(): #X按钮
    # 机器人身体前移100，以100速度朝前
    pd.set_body_pos(0, 100, 50)
    pd.set_velocity(200, 0, 0)

def set_PSB_START_button(): #START按钮
    pass

def set_LEFT_STICK_button(lx1, ly1): #左摇杆
    # 计算速度

    vx = lx1 * pd.MAX_SPEED
    vy = -ly1 * pd.MAX_SPEED
    # 设置速度
    pd.set_velocity(vx, vy, 0)

def set_RIGHT_STICK_button(lx2, ly2): #右摇杆
    
    w = lx2 * pd.MAX_SPEED
    pd.set_velocity(0, 0, w)
    
#十字键
def set_Right_button(): #右
    pd.MAX_R_PACE = 60
    pd.set_velocity(0, -200, 0)
def set_Left_button(): #左
    pd.MAX_R_PACE = 60
    pd.set_velocity(0, 200, 0)
def set_Up_button(): #上
    pd.set_velocity(200, 0, 0)
def set_Down_button(): #下
    pd.set_velocity(-200, 0, 0)


# 按钮松开时执行的函数
def button_release():
    pd.MAX_R_PACE = 80
    # pd.set_body_pos(0, 0, 0)
    pd.set_velocity(0, 0, 0)


