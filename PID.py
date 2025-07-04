import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
import matplotlib.gridspec as gridspec
import matplotlib as mpl
import matplotlib
matplotlib.use('TkAgg')

# 设置全局样式
mpl.rcParams['font.size'] = 12
mpl.rcParams['axes.titlesize'] = 14
mpl.rcParams['axes.labelsize'] = 12
mpl.rcParams['xtick.labelsize'] = 10
mpl.rcParams['ytick.labelsize'] = 10


class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=1.0):
        self.Kp = Kp  # 比例增益
        self.Ki = Ki  # 积分增益
        self.Kd = Kd  # 微分增益
        self.setpoint = setpoint  # 目标值
        self.prev_error = 0  # 上一次误差
        self.integral = 0  # 积分累积
        self.max_integral = 5  # 积分限幅
        self.output_limits = (-5, 5)  # 输出限幅

    def update(self, current_value, dt=0.1):
        # 计算误差
        error = self.setpoint - current_value

        # 比例项
        P = self.Kp * error

        # 积分项（带限幅）
        self.integral += error * dt
        # 积分抗饱和
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        I = self.Ki * self.integral

        # 微分项
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        self.prev_error = error

        # 计算总输出
        output = P + I + D

        # 输出限幅
        if output > self.output_limits[1]:
            output = self.output_limits[1]
        elif output < self.output_limits[0]:
            output = self.output_limits[0]

        return output, (P, I, D, error)


class SystemSimulator:
    def __init__(self):
        self.value = 0.0  # 当前系统值
        self.time = 0.0  # 当前时间
        self.noise_level = 0.0  # 噪声水平
        self.saturation = True  # 是否启用饱和限制

    def update(self, input_val, dt=0.1):
        # 系统动力学模型
        # 简单的一阶系统：dy/dt = input - 0.2*y
        self.value += (input_val - 0.2 * self.value) * dt

        # 添加噪声
        if self.noise_level > 0:
            noise = np.random.normal(0, self.noise_level)
            self.value += noise

        # 应用饱和限制
        if self.saturation:
            if self.value > 10:
                self.value = 10
            elif self.value < -10:
                self.value = -10

        self.time += dt
        return self.value


# 创建图形和布局
fig = plt.figure(figsize=(14, 12), facecolor='#f5f5f5')
fig.suptitle('Interactive PID Controller Simulation', fontsize=18, fontweight='bold', y=0.97)

# 使用GridSpec创建更灵活的布局
gs = gridspec.GridSpec(6, 4, figure=fig, height_ratios=[1.2, 1, 1, 0.3, 0.3, 0.3])

# 创建子图
ax1 = fig.add_subplot(gs[0:2, :])  # 系统响应
ax2 = fig.add_subplot(gs[2, :])  # PID分量
ax3 = fig.add_subplot(gs[3, :])  # 误差
ax_params = fig.add_subplot(gs[4, :])  # 参数滑块
ax_controls = fig.add_subplot(gs[5, :])  # 控制按钮

# 隐藏参数和控制区域的轴
ax_params.axis('off')
ax_controls.axis('off')

# 设置初始参数
init_Kp = 0.5
init_Ki = 0.1
init_Kd = 0.05
init_setpoint = 1.0
init_noise = 0.0
init_disturbance = False
init_saturation = True

# 创建滑块区域
slider_width = 0.3
slider_height = 0.03
slider_padding = 0.05

# Kp滑块
ax_Kp = fig.add_axes([0.15, 0.15, slider_width, slider_height], facecolor='#e6e6e6')
s_Kp = Slider(ax_Kp, 'Proportional Gain (Kp)', 0.0, 5.0, valinit=init_Kp,
              color='#4c72b0', track_color='#d0d0d0')

# Ki滑块
ax_Ki = fig.add_axes([0.15, 0.10, slider_width, slider_height], facecolor='#e6e6e6')
s_Ki = Slider(ax_Ki, 'Integral Gain (Ki)', 0.0, 1.0, valinit=init_Ki,
              color='#55a868', track_color='#d0d0d0')

# Kd滑块
ax_Kd = fig.add_axes([0.15, 0.05, slider_width, slider_height], facecolor='#e6e6e6')
s_Kd = Slider(ax_Kd, 'Derivative Gain (Kd)', 0.0, 1.0, valinit=init_Kd,
              color='#c44e52', track_color='#d0d0d0')

# Setpoint滑块
ax_setpoint = fig.add_axes([0.55, 0.15, slider_width, slider_height], facecolor='#e6e6e6')
s_setpoint = Slider(ax_setpoint, 'Setpoint', -2.0, 2.0, valinit=init_setpoint,
                    color='#8172b2', track_color='#d0d0d0')

# Noise滑块
ax_noise = fig.add_axes([0.55, 0.10, slider_width, slider_height], facecolor='#e6e6e6')
s_noise = Slider(ax_noise, 'Noise Level', 0.0, 0.5, valinit=init_noise,
                 color='#ccb974', track_color='#d0d0d0')

# 创建复选框
disturbance_box_ax = fig.add_axes([0.55, 0.05, 0.15, 0.03])
disturbance_box = CheckButtons(disturbance_box_ax, ['Add Disturbance'], [init_disturbance])

saturation_box_ax = fig.add_axes([0.75, 0.05, 0.15, 0.03])
saturation_box = CheckButtons(saturation_box_ax, ['Saturation'], [init_saturation])

# 初始化数据
time_points = np.linspace(0, 50, 500)
actual_values = np.zeros_like(time_points)
setpoint_values = np.zeros_like(time_points)
P_vals = np.zeros_like(time_points)
I_vals = np.zeros_like(time_points)
D_vals = np.zeros_like(time_points)
errors = np.zeros_like(time_points)


# 绘图函数
def update_plots():
    # 重置模拟
    pid = PIDController(s_Kp.val, s_Ki.val, s_Kd.val, s_setpoint.val)
    system = SystemSimulator()
    system.noise_level = s_noise.val

    # 获取复选框状态
    system.saturation = saturation_box.get_status()[0]
    disturbance_active = disturbance_box.get_status()[0]

    # 运行模拟
    for i, t in enumerate(time_points):
        # 在t=20时添加干扰
        if disturbance_active and 20 < t < 25:
            system.value += 0.5

        setpoint_values[i] = pid.setpoint
        actual_values[i] = system.value
        output, (P, I, D, error) = pid.update(system.value)
        system.update(output)

        P_vals[i] = P
        I_vals[i] = I
        D_vals[i] = D
        errors[i] = error

    # 更新图表
    ax1.clear()
    ax1.plot(time_points, setpoint_values, 'r--', label='Setpoint', linewidth=2)
    ax1.plot(time_points, actual_values, 'b-', label='System Response', linewidth=1.8)

    # 添加干扰区域
    if disturbance_active:
        ax1.axvspan(20, 25, alpha=0.2, color='red', label='Disturbance')

    ax1.set_title('System Response', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Value')
    ax1.legend(loc='best', frameon=True, shadow=True)
    ax1.grid(True, linestyle='--', alpha=0.7)

    # 添加稳态误差标注
    if abs(pid.setpoint - actual_values[-1]) > 0.1:
        ax1.annotate(f'Steady-State Error: {pid.setpoint - actual_values[-1]:.2f}',
                     xy=(time_points[-1], actual_values[-1]),
                     xytext=(time_points[-1] - 10, actual_values[-1] + 0.5),
                     arrowprops=dict(facecolor='red', shrink=0.05, width=2),
                     fontsize=10, color='red', weight='bold')

    # 添加积分饱和标注
    if abs(pid.integral) >= pid.max_integral - 0.1:
        idx = np.argmax(np.abs(I_vals))
        ax1.annotate('Integral Saturation!',
                     xy=(time_points[idx], actual_values[idx]),
                     xytext=(15, 1.5),
                     arrowprops=dict(facecolor='purple', shrink=0.05, width=2),
                     fontsize=10, color='purple', weight='bold')

    # 添加饱和区域
    if system.saturation:
        ax1.axhspan(-10, -9.5, alpha=0.1, color='gray')
        ax1.axhspan(9.5, 10, alpha=0.1, color='gray')

    # 更新PID分量图表
    ax2.clear()
    ax2.plot(time_points, P_vals, 'g-', label='Proportional', linewidth=1.8)
    ax2.plot(time_points, I_vals, 'b-', label='Integral', linewidth=1.8)
    ax2.plot(time_points, D_vals, 'r-', label='Derivative', linewidth=1.8)
    ax2.set_title('PID Components', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Component Value')
    ax2.legend(loc='best', frameon=True, shadow=True)
    ax2.grid(True, linestyle='--', alpha=0.7)

    # 添加噪声敏感标注
    if s_noise.val > 0.1 and abs(np.std(D_vals)) > 0.5:
        max_d_idx = np.argmax(np.abs(D_vals))
        ax2.annotate('Noise Amplified by Derivative!',
                     xy=(time_points[max_d_idx], D_vals[max_d_idx]),
                     xytext=(time_points[max_d_idx] - 10, D_vals[max_d_idx] + 0.5),
                     arrowprops=dict(facecolor='orange', shrink=0.05, width=2),
                     fontsize=10, color='orange', weight='bold')

    # 更新误差图表
    ax3.clear()
    ax3.plot(time_points, errors, 'm-', label='Error', linewidth=1.8)
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_title('Error', fontsize=14, fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Error')
    ax3.legend(loc='best', frameon=True, shadow=True)
    ax3.grid(True, linestyle='--', alpha=0.7)

    # 添加背景色
    for ax in [ax1, ax2, ax3]:
        ax.set_facecolor('#f9f9f9')

    fig.canvas.draw_idle()


# 初始更新
update_plots()


# 更新函数
def update(val):
    update_plots()


# 注册更新函数
s_Kp.on_changed(update)
s_Ki.on_changed(update)
s_Kd.on_changed(update)
s_setpoint.on_changed(update)
s_noise.on_changed(update)


# 复选框的回调函数
def disturbance_changed(label):
    update_plots()


def saturation_changed(label):
    update_plots()


disturbance_box.on_clicked(disturbance_changed)
saturation_box.on_clicked(saturation_changed)

# 添加重置按钮
resetax = fig.add_axes([0.75, 0.15, 0.15, 0.04])
button = Button(resetax, 'Reset to Default', color='#e6e6e6', hovercolor='#d0d0d0')
button.label.set_fontsize(10)


def reset(event):
    s_Kp.reset()
    s_Ki.reset()
    s_Kd.reset()
    s_setpoint.reset()
    s_noise.reset()

    # 重置复选框状态
    if disturbance_box.get_status()[0]:
        disturbance_box.set_active(0)
    if not saturation_box.get_status()[0]:
        saturation_box.set_active(0)

    update_plots()


button.on_clicked(reset)

# 添加解释框
explanation_text = (
    "PID Controller Issues:\n\n"
    "1. Proportional (P): Steady-state error\n"
    "   - System doesn't reach setpoint with only P control\n\n"
    "2. Integral (I): Integral saturation\n"
    "   - Accumulated error causes overshoot and oscillation\n\n"
    "3. Derivative (D): Noise sensitivity\n"
    "   - Amplifies measurement noise in the system"
)

props = dict(boxstyle='round', facecolor='white', alpha=0.8, edgecolor='gray')
ax1.text(0.97, 0.97, explanation_text,
         transform=ax1.transAxes, fontsize=10,
         verticalalignment='top', horizontalalignment='right',
         bbox=props)

# 添加设计元素
# 在顶部添加装饰条
fig.patches.append(plt.Rectangle((0, 0.94), 1, 0.03,
                                 transform=fig.transFigure,
                                 facecolor='#4c72b0',
                                 edgecolor='none',
                                 alpha=0.8,
                                 zorder=100))

# 在滑块区域添加标题
ax_params.text(0.5, 0.95, 'PID Parameters',
               transform=ax_params.transAxes,
               fontsize=12, fontweight='bold',
               horizontalalignment='center',
               verticalalignment='center')

# 调整布局
plt.subplots_adjust(left=0.08, right=0.95, top=0.92, bottom=0.18, hspace=0.6)

plt.show()