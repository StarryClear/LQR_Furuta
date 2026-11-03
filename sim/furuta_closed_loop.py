# -*- coding: utf-8 -*-
"""
furuta_closed_loop.py -- 用实测参数复刻 LQR_2.1（起摆 + LQR 平衡）的定点实现，
                       在非线性 Furuta 模型上跑闭环，用来在烧板之前验证：
                         1) 能量泵方向（invert_pd）对不对
                         2) 捕获窗口 / 能量阈值能不能真的抓到摆
                         3) 交接后 LQR 能不能站住（含 phi 外环、死区前馈、积分）

控制侧全部按 RTL 的整数运算逐步复刻（shift / clamp / 状态机）；
被控对象用 CALIBRATION.md / FINDINGS.md 的实测参数（m=0.14, l=0.0756, r=0.128,
M11=1.070e-3, Ja=1.8e-3, bp=8.89e-5, K_stall=0.216, w_nl=33.5），
臂侧输入增益用实测 B = 0.019 rad/s^2 per duty count（F-07）。

用法（本机没有 py，用 DSH 自带 python）：
    python furuta_closed_loop.py            # 默认场景
    python furuta_closed_loop.py --swp      # 扫 invert_pd / E_STOP
"""
import math
import sys

# ----------------------------------------------------------------------
# 0. 实测物理参数
# ----------------------------------------------------------------------
M_P   = 0.14       # 摆杆质量 kg
L_P   = 0.0756     # 枢轴 -> 质心 m
R_ARM = 0.128      # 臂长 m
GRAV  = 9.81
M11   = 1.070e-3   # 摆杆绕枢轴转动惯量（自由摆动实测）
Ja    = 1.80e-3    # 臂侧总惯量（CALIBRATION 用值）
bp    = 8.89e-5    # 摆杆阻尼 N·m·s/rad
K_ST  = 0.216      # 堵转力矩 N·m
W_NL  = 33.5       # 空载转速 rad/s
B_ARM = 0.019      # 实测：臂角加速度 rad/s^2 per duty count
DUTY_MAX = 2500

M22 = Ja + M_P * R_ARM * R_ARM
M12 = M_P * R_ARM * L_P
DET = M11 * M22 - M12 * M12
U_PER_DUTY = B_ARM * DET / M11          # 每个 duty count 对应的 N·m
BA = K_ST / W_NL                        # 反电动势等效阻尼
DZ_P = 113.0
DZ_N = 167.0
TORQUE_SIGN = -1.0    # +duty -> 负力矩（实测编码器方向）

# ----------------------------------------------------------------------
# 1. 被控对象：x = [th, thd, ph, phd]，th=0 竖直向上，ph 臂角
# ----------------------------------------------------------------------
def deriv(th, thd, ph, phd, u):
    c, s = math.cos(th), math.sin(th)
    a11 = M11
    a12 = M12 * c
    a22 = Ja + M_P * R_ARM * R_ARM + M_P * L_P * L_P * s * s
    det = a11 * a22 - a12 * a12
    r1 = M_P * L_P * L_P * s * c * phd * phd + M_P * GRAV * L_P * s - bp * thd
    r2 = (-2.0 * M_P * L_P * L_P * s * c * thd * phd
          + M_P * R_ARM * L_P * s * thd * thd - BA * phd + u)
    thdd = (a22 * r1 - a12 * r2) / det
    phdd = (-a12 * r1 + a11 * r2) / det
    return thd, thdd, phd, phdd


def rk4(th, thd, ph, phd, u, dt):
    k1 = deriv(th, thd, ph, phd, u)
    k2 = deriv(th + 0.5 * dt * k1[0], thd + 0.5 * dt * k1[1],
               ph + 0.5 * dt * k1[2], phd + 0.5 * dt * k1[3], u)
    k3 = deriv(th + 0.5 * dt * k2[0], thd + 0.5 * dt * k2[1],
               ph + 0.5 * dt * k2[2], phd + 0.5 * dt * k2[3], u)
    k4 = deriv(th + dt * k3[0], thd + dt * k3[1],
               ph + dt * k3[2], phd + dt * k3[3], u)
    return (th + dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
            thd + dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
            ph + dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]),
            phd + dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]))


def duty_torque(duty):
    """duty(counts) -> 真实力矩 N·m（含死区，不含反电动势，反电动势在 EOM 里）

    TORQUE_SIGN = -1 对应实测方向：+duty 让编码器 counts 减小（CALIBRATION.md），
    即 +duty 在模型坐标里是负力矩。模型与硬件是否一致，用"已知能站住的 LQR"
    做判据：把这个符号翻对，LQR 才会收敛。
    """
    if duty == 0:
        return 0.0
    dz = DZ_P if duty > 0 else DZ_N
    d = abs(duty) - dz
    if d < 0:
        d = 0.0
    return TORQUE_SIGN * math.copysign(U_PER_DUTY * d, duty)


# ----------------------------------------------------------------------
# 2. swing_ctrl.v 的逐拍复刻
# ----------------------------------------------------------------------
COS_LUT = [
    32767, 32717, 32564, 32309, 31954, 31499, 30945, 30296, 29551, 28715,
    27789, 26776, 25680, 24504, 23251, 21926, 20533, 19075, 17558, 15986,
    14365, 12698, 10992, 9252, 7483, 5690, 3880, 2058, 229, -1601,
    -3425, -5239, -7036, -8812, -10560, -12275, -13952, -15585, -17170, -18701,
    -20174, -21584, -22927, -24198, -25393, -26510, -27544, -28492, -29351, -30118,
    -30792, -31369, -31849, -32230, -32510, -32688, -32765, -32768, -32768, -32768,
    -32768, -32768, -32768, -32768,
]


def sgn_ext(v, bits=16):
    """把整数按 bits 位有符号解释"""
    v &= (1 << bits) - 1
    if v >= (1 << (bits - 1)):
        v -= (1 << bits)
    return v


def ash(v, n):
    """算术右移（Verilog >>> ）"""
    return v >> n


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class SwingCtrl:
    S_IDLE, S_KICK1, S_KICK2, S_PUMP, S_WAIT, S_BAL, S_FALL = range(7)

    def __init__(self, frame_hz=856, L_mm=76, E_K_Q8=None, e_lpf=3, invert_pd=1,
                 E_STOP_Q=-2000, E_RESUME_Q=-6554,
                 ANG_CAP=220, W_CAP=120,
                 I_KICK=1300, K_MA=6, I_FLOOR=700, I_MAX=1500,
                 kick_ms=250, wait_ms=150, stall_ms=300, escalate_ms=2500,
                 rest_ms=400, settle_timeout_ms=6000, timeout_ms=30000,
                 auto_reswing=1, sw_sign=1):
        self.frame_hz = frame_hz
        self.L_mm = L_mm
        self.invert_pd = invert_pd
        self.E_STOP_Q = E_STOP_Q
        self.E_RESUME_Q = E_RESUME_Q
        self.ANG_CAP = ANG_CAP
        self.W_CAP = W_CAP
        self.I_KICK = I_KICK
        self.K_MA = K_MA
        self.I_FLOOR_START = I_FLOOR
        self.I_FLOOR_MAX = I_MAX
        self.I_MAX = I_MAX
        self.KICK_FRAMES = frame_hz * kick_ms // 1000
        self.WAIT_FRAMES = frame_hz * wait_ms // 1000
        self.STALL_FRAMES = frame_hz * stall_ms // 1000
        self.ESCALATE_FRAMES = frame_hz * escalate_ms // 1000
        self.REST_FRAMES = frame_hz * rest_ms // 1000
        self.SETTLE_TIMEOUT_FRAMES = frame_hz * settle_timeout_ms // 1000
        self.TIMEOUT_FRAMES = frame_hz * timeout_ms // 1000
        self.E_K_Q8 = (L_mm * 133 + 512) >> 10 if E_K_Q8 is None else E_K_Q8
        self.E_LPF = e_lpf
        self.AUTO_RESWING = auto_reswing
        self.SW_SIGN = sw_sign
        self.reset()

    def reset(self):
        self.state = self.S_IDLE
        self.ang_r = 0
        self.ang_prev = 0
        self.d_r = 0
        self.raw_r = 0
        self.spd_f = 0
        self.spd_q = 0
        self.cos_q_r = 0
        self.e_kin_r = 0
        self.e_q = 0
        self.e_f = 0
        self.i_pump_r = 0
        self.i_level = self.I_FLOOR_START
        self.ms_cnt = self.sw_ms = self.stall_cnt = self.esc_cnt = self.rest_cnt = 0
        self.kick_dir = 0
        self.run_d = 0
        self.fall_cnt = 0
        self.cap_cnt = 0
        self.spd_clear = 0
        self.bal_entry = 0
        self.force_fall = 0

    # ---- 单拍（每个 angle frame 调用一次）----
    def step(self, run_en, theta10, arm_speed10, force_fall=0):
        self.force_fall = force_fall
        self.bal_entry = 0
        frame = True
        # --- 快速角速度（帧差分 + 1/2 低通）---
        self.ang_r = theta10
        d_ang = self.ang_r - self.ang_prev
        if d_ang > 1800:
            d_ang -= 3600
        elif d_ang < -1800:
            d_ang += 3600
        self.ang_prev = self.ang_r
        raw_c = self.d_r * 86
        spd_nxt = self.spd_f + ash(self.raw_r - self.spd_f, 1)
        if self.spd_clear:
            self.d_r = 0
            self.raw_r = 0
            self.spd_f = 0
            self.spd_q = 0
        else:
            self.d_r = d_ang
            self.raw_r = raw_c
            self.spd_f = spd_nxt
            self.spd_q = clip(spd_nxt, -3000, 3000)

        # --- cos / 能量 ---
        ang_u = abs(self.ang_r)
        idx = 56 if ang_u > 1792 else (ang_u >> 5)
        cos_q = COS_LUT[idx]
        spd_sq = self.spd_q * self.spd_q
        self.e_kin_r = ash(spd_sq * self.E_K_Q8, 8)
        self.cos_q_r = cos_q
        self.e_q = self.e_kin_r + self.cos_q_r - 32768
        self.e_f = self.e_f + ((self.e_q - self.e_f) >> self.E_LPF)

        # --- 泵能电流 ---
        w_neg = 1 if self.spd_q < 0 else 0
        cos_neg = 1 if ang_u > 900 else 0
        g_neg = self.invert_pd ^ (1 if self.SW_SIGN < 0 else 0)
        u_neg = ~(g_neg ^ (w_neg ^ cos_neg)) & 1
        spd_abs = -self.spd_q if w_neg else self.spd_q
        mag_raw = spd_abs * self.K_MA
        mag_lim = self.I_MAX if mag_raw > self.I_MAX else mag_raw
        mag_flr = (max(mag_lim, self.i_level) if spd_abs > 20 else mag_lim)
        mag_use = mag_flr if ang_u > 900 else mag_lim
        i_pump_c = -mag_use if u_neg else mag_use
        self.i_pump_r = i_pump_c

        # --- 捕获判据 ---
        cap_raw = (-self.ANG_CAP < self.ang_r < self.ANG_CAP) and \
                  (-self.W_CAP < self.spd_q < self.W_CAP)
        self.cap_cnt = min(2, self.cap_cnt + 1) if cap_raw else 0
        cap_ok = cap_raw and self.cap_cnt >= 1

        # --- 计时 ---
        sw_active = run_en and self.state in (self.S_KICK1, self.S_KICK2,
                                              self.S_PUMP, self.S_WAIT, self.S_FALL)
        if not sw_active:
            self.sw_ms = 0
            self.esc_cnt = 0
        else:
            if self.sw_ms < self.TIMEOUT_FRAMES:
                self.sw_ms += 1
            if self.state == self.S_PUMP:
                if self.esc_cnt < self.ESCALATE_FRAMES:
                    self.esc_cnt += 1
                else:
                    self.esc_cnt = 0
                    self.i_level = min(self.I_FLOOR_MAX, self.i_level + 100)
            else:
                self.esc_cnt = 0

        if self.state != self.S_BAL:
            self.fall_cnt = 0

        # --- 状态机 ---
        st = self.state
        if st == self.S_IDLE:
            self.ms_cnt = 0
            self.i_level = self.I_FLOOR_START
            self.kick_dir = 0
            if run_en and not self.run_d:
                if cap_ok:
                    self.state = self.S_BAL
                    self.bal_entry = 1
                else:
                    self.state = self.S_KICK1
        elif st == self.S_KICK1:
            if not run_en:
                self.state = self.S_IDLE
            else:
                if self.ms_cnt >= self.KICK_FRAMES - 1:
                    self.ms_cnt = 0
                    self.state = self.S_KICK2
                else:
                    self.ms_cnt += 1
        elif st == self.S_KICK2:
            if not run_en:
                self.state = self.S_IDLE
            else:
                if self.ms_cnt >= self.KICK_FRAMES - 1:
                    self.ms_cnt = 0
                    self.stall_cnt = 0
                    self.state = self.S_PUMP
                else:
                    self.ms_cnt += 1
        elif st == self.S_PUMP:
            if not run_en:
                self.state = self.S_IDLE
            elif self.sw_ms >= self.TIMEOUT_FRAMES:
                self.state = self.S_IDLE
            elif cap_ok:
                self.state = self.S_BAL
                self.bal_entry = 1
            elif self.e_f > self.E_STOP_Q:
                self.ms_cnt = 0
                self.state = self.S_WAIT
            else:
                if -25 < self.spd_q < 25:
                    if self.stall_cnt >= self.STALL_FRAMES:
                        self.stall_cnt = 0
                        self.ms_cnt = 0
                        self.kick_dir ^= 1
                        self.i_level = min(self.I_FLOOR_MAX, self.i_level + 100)
                        self.state = self.S_KICK1
                    else:
                        self.stall_cnt += 1
                else:
                    self.stall_cnt = 0
        elif st == self.S_WAIT:
            if not run_en:
                self.state = self.S_IDLE
            elif self.sw_ms >= self.TIMEOUT_FRAMES:
                self.state = self.S_IDLE
            elif cap_ok:
                self.state = self.S_BAL
                self.bal_entry = 1
            else:
                if self.ms_cnt < self.WAIT_FRAMES:
                    self.ms_cnt += 1
                elif self.e_f < self.E_RESUME_Q or ang_u > 1200:
                    self.state = self.S_PUMP
        elif st == self.S_BAL:
            if not run_en:
                self.state = self.S_IDLE
            elif force_fall:
                self.i_level = self.I_FLOOR_START
                self.stall_cnt = self.esc_cnt = self.rest_cnt = 0
                self.ms_cnt = 0
                self.state = self.S_FALL
            elif ang_u > 600:
                if self.fall_cnt >= 2:
                    if self.AUTO_RESWING:
                        self.i_level = self.I_FLOOR_START
                        self.stall_cnt = self.esc_cnt = self.rest_cnt = 0
                        self.ms_cnt = 0
                        self.state = self.S_FALL
                    else:
                        self.state = self.S_IDLE
                else:
                    self.fall_cnt += 1
        elif st == self.S_FALL:
            if not run_en:
                self.state = self.S_IDLE
            else:
                self.ms_cnt += 1
                if ang_u > 1600 and -50 < self.spd_q < 50 and -1500 < arm_speed10 < 1500:
                    self.rest_cnt += 1
                else:
                    self.rest_cnt = 0
                if self.rest_cnt >= self.REST_FRAMES - 1:
                    self.ms_cnt = self.rest_cnt = self.stall_cnt = self.esc_cnt = 0
                    self.i_level = self.I_FLOOR_START
                    self.kick_dir = 0
                    self.spd_clear = 1
                    self.state = self.S_KICK1
                elif self.ms_cnt >= self.SETTLE_TIMEOUT_FRAMES - 1:
                    self.ms_cnt = self.rest_cnt = self.stall_cnt = self.esc_cnt = 0
                    self.i_level = self.I_FLOOR_START
                    self.kick_dir = 0
                    self.spd_clear = 1
                    self.state = self.S_KICK1
        self.run_d = 1 if run_en else 0
        # 下一拍的 spd_clear 是单拍
        # （RTL 里在 always 开头清 0，这里在本拍末尾清，等价）

        # --- 输出电流 ---
        if not run_en:
            i_sw = 0
        elif self.state == self.S_KICK1:
            i_sw = -self.I_KICK if self.kick_dir else self.I_KICK
        elif self.state == self.S_KICK2:
            i_sw = self.I_KICK if self.kick_dir else -self.I_KICK
        elif self.state == self.S_PUMP:
            i_sw = self.i_pump_r
        else:
            i_sw = 0
        self.spd_clear = 0
        return i_sw


# ----------------------------------------------------------------------
# 3. lqr_core.v 的逐毫秒复刻
# ----------------------------------------------------------------------
class LQRCore:
    def __init__(self, KTH=4600, KDT=60, KI=2, KDP=75, KPH=-30,
                 TH_LIMIT=300, DZ_P=113, DZ_N=167, limit=2000,
                 I_LIM=100000, TH_TGT_LIM=50):
        self.KTH, self.KDT, self.KI, self.KDP, self.KPH = KTH, KDT, KI, KDP, KPH
        self.TH_LIMIT, self.DZ_P, self.DZ_N = TH_LIMIT, DZ_P, DZ_N
        self.limit, self.I_LIM, self.TH_TGT_LIM = limit, I_LIM, TH_TGT_LIM
        self.h = [0] * 5
        self.integ = 0
        self.phi0 = 0
        self.en_d = 0
        self.init_cnt = 0
        self.ready = 0
        self.sub5 = 0
        self.thetad10 = 0
        self.duty = 0
        self.u_raw = 0
        # phi 外环流水
        self.dphi_r = 0
        self.dphi_x32 = 0
        self.dphi_x2 = 0
        self.th_tgt = 0
        self.th_err = 0
        # kdp 流水
        self.k64 = self.k8 = self.k2 = self.k1 = 0
        self.kdp = 0
        # 诊断
        self.dbg = {}

    def tick(self, theta10, phi10, phid10, enable, sa_lost=False):
        if not enable:
            self.thetad10 = 0
            self.duty = 0
            self.u_raw = 0
            self.init_cnt = 0
            self.ready = 0
            self.sub5 = 0
            self.integ = 0
            self.en_d = 0
            self.h = [theta10] * 5
            self.dbg = dict(theta=theta10, th_tgt=0, th_err=0, u_p=0, integ=0,
                            thetad=0, dphi=0)
            return self.duty

        # 4ms 基线差分（用本拍之前的 h[4]）
        dth4 = (theta10 - self.h[4]) * 250

        in_range = (not sa_lost) and (-self.TH_LIMIT <= theta10 <= self.TH_LIMIT) and self.ready

        dphi_w = phi10 - self.phi0
        if dphi_w > 1800:
            dphi = dphi_w - 3600
        elif dphi_w < -1800:
            dphi = dphi_w + 3600
        else:
            dphi = dphi_w

        # 历史移位 / 速度
        self.h = [theta10] + self.h[:4]
        if self.init_cnt != 8:
            self.init_cnt += 1
            self.thetad10 = 0
            self.ready = 0
        else:
            self.ready = 1
            self.thetad10 = clip(dth4, -20000, 20000)

        if not self.en_d:
            self.phi0 = phi10
        self.en_d = 1

        # ---- 外环流水（2 拍）----
        th_tgt_raw = (self.dphi_x2 - self.dphi_x32) >> 8
        self.dphi_r = dphi
        self.dphi_x32 = self.dphi_r << 5
        self.dphi_x2 = self.dphi_r << 1
        tgt = clip(th_tgt_raw, -self.TH_TGT_LIM, self.TH_TGT_LIM)
        self.th_tgt = tgt
        self.th_err = theta10 - self.th_tgt

        # ---- KDP 流水（1 拍）----
        self.k64 = phid10 << 6
        self.k8 = phid10 << 3
        self.k2 = phid10 << 1
        self.k1 = phid10
        self.kdp = self.k64 + self.k8 + self.k2 + self.k1

        # ---- 积分 ----
        if in_range:
            if self.integ > self.I_LIM:
                self.integ = self.I_LIM
            elif self.integ < -self.I_LIM:
                self.integ = -self.I_LIM
            else:
                self.integ += theta10
        else:
            self.integ = 0

        prod = (self.KTH * self.th_err + self.KI * self.integ +
                self.KDT * self.thetad10 + self.kdp)
        u_p = -(prod >> 8)
        if u_p > 0:
            u_c = u_p + self.DZ_P
        elif u_p < 0:
            u_c = u_p - self.DZ_N
        else:
            u_c = 0
        self.dbg = dict(theta=theta10, th_tgt=self.th_tgt, th_err=self.th_err,
                        u_p=u_p, integ=self.integ, thetad=self.thetad10,
                        dphi=dphi, kdp=self.kdp, phi0=self.phi0)
        if not in_range:
            self.duty = 0
            self.u_raw = 0
        else:
            self.u_raw = clip(u_c, -30000, 30000)
            self.duty = clip(u_c, -self.limit, self.limit)
        return self.duty


# ----------------------------------------------------------------------
# 4. 顶层拼接 + 仿真
# ----------------------------------------------------------------------
class DesignLQR:
    """用 CALIBRATION.md 里那组 LQR 增益 K=[-3.21,-0.31,-0.32,-0.18]（u=-Kx）的
    "理想平衡内核"。它在本模型里是稳定的；固件里那组 KTH/KDT/KPH/KDP 是针对实物
    摩擦/回差手调出来的，在无摩擦模型里不收敛（见 README 的说明），所以仿真用这组
    只为验证"起摆 -> 捕获 -> 交接"这段逻辑，不用于核对固件增益。

    结构与固件一致：1 kHz、4 ms 基线差分估 θ̇、φ 用交接瞬间的 φ0 作零点、±2000 限幅、
    死区前馈 113/167。"""

    def __init__(self, K=(3.21, 0.31, 0.32, 0.18), limit=2000, DZ_P=113, DZ_N=167):
        self.K = K
        self.limit = limit
        self.DZ_P = DZ_P
        self.DZ_N = DZ_N
        self.reset()

    def reset(self):
        self.h = [0] * 5
        self.phi0 = None
        self.en_d = False
        self.duty = 0
        self.thetad = 0.0
        self.dbg = dict(theta=0, th_tgt=0, th_err=0, u_p=0, integ=0, thetad=0, dphi=0)
        self.integ = 0

    def tick(self, theta10, phi10, phid10, enable, sa_lost=False):
        if not enable:
            self.reset()
            return 0
        th = math.radians(theta10 / 10.0)
        if not self.en_d:
            self.phi0 = phi10
            self.h = [theta10] * 5
        self.en_d = True
        dth4 = (theta10 - self.h[4]) * 250
        self.h = [theta10] + self.h[:4]
        thd = clip(dth4, -20000, 20000) / 572.96
        dphi = phi10 - self.phi0
        if dphi > 1800:
            dphi -= 3600
        elif dphi < -1800:
            dphi += 3600
        dphi = math.radians(dphi / 10.0)
        phid = math.radians(phid10 / 10.0)
        self.thetad = thd
        u = self.K[0] * th + self.K[1] * thd + self.K[2] * dphi + self.K[3] * phid
        duty = -u / U_PER_DUTY
        if duty > 0:
            duty += self.DZ_P
        elif duty < 0:
            duty -= self.DZ_N
        self.duty = clip(int(duty), -self.limit, self.limit)
        self.dbg = dict(theta=theta10, th_tgt=0, th_err=0, u_p=duty, integ=0,
                        thetad=thd, dphi=dphi)
        return self.duty


def run(swing_kw=None, lqr_kw=None, t_end=30.0, dt=100e-6, verbose=True,
        theta0_deg=180.0, perturb_deg=0.0, run_delay=0.10, mode='auto',
        ctrl='fw'):
    sw = SwingCtrl(**(swing_kw or {}))
    if ctrl == 'design':
        lq = DesignLQR(**(lqr_kw or {}))
    else:
        lq = LQRCore(**(lqr_kw or {}))
    th = math.radians(theta0_deg + perturb_deg)
    thd = 0.0
    ph = 0.0
    phd = 0.0

    frame_period = 1.0 / sw.frame_hz
    t_frame = frame_period
    t_tick = 0.001
    t_enc = 0.010

    # 传感器 / 遥测
    theta10 = -1800
    phi10 = 0
    phid10 = 0
    pos_counts = 0
    pos_counts_prev = 0
    spd_counts = 0
    duty = 0
    sw_i = 0
    sw_duty = 0
    import random
    random.seed(12345)

    log = []
    t = 0.0
    lqr_on = False
    force_fall = False
    force_fall_cnt = 0
    bal_time = 0.0
    max_abs_th_after_bal = 0.0
    bal_start = None
    first_fall = None
    states = []

    while t < t_end:
        run_en = 1 if t >= run_delay else 0

        # ---- 角度帧（856Hz）----
        while t >= t_frame:
            t_frame += frame_period
            # SA100 量化 + 噪声
            pos_raw = int(round((math.degrees(th) + 3600.0) % 3600.0 * 4096.0 / 360.0 +
                                1330.0)) % 4096
            noise = int(round(random.gauss(0.0, 0.9)))   # 0.09° rms -> 0.1° 单位
            pos_raw = (pos_raw + noise) % 4096
            d = (pos_raw - 1330) % 4096
            if d >= 2048:
                d -= 4096
            theta10 = ash(d * 225, 8)
            # swing
            if mode == 'balance':
                sw.state = 5 if run_en else 0
                sw_i = 0
            else:
                sw_i = sw.step(run_en, theta10, phid10, force_fall=1 if force_fall else 0)
            # mA -> duty (×1.5, 限幅)
            sw_duty = clip((sw_i * 3) >> 1, -2000, 2000)
            # 掉摆判据（顶层）：state==5 且 |θ|>25° 连续 2 帧
            if sw.state == 5 and abs(theta10) > 250:
                force_fall_cnt += 1
                if force_fall_cnt >= 2:
                    force_fall = True
                else:
                    force_fall = False
            else:
                force_fall_cnt = 0
                force_fall = False
            states.append(sw.state)

        # ---- 1 kHz LQR ----
        if t >= t_tick:
            t_tick += 0.001
            lqr_on = run_en and (sw.state == 5)
            duty_lqr = lq.tick(theta10, phi10, phid10, lqr_on)
            duty = duty_lqr if lqr_on else (sw_duty if run_en else 0)
            if lqr_on and bal_start is None:
                bal_start = t
            if lqr_on:
                bal_time += 0.001
                max_abs_th_after_bal = max(max_abs_th_after_bal, abs(math.degrees(th)))
                if first_fall is None and abs(math.degrees(th)) > 30.0:
                    first_fall = t - bal_start if bal_start is not None else 0.0

        # ---- 10 ms 编码器测速 ----
        if t >= t_enc:
            t_enc += 0.010
            pos_counts = int(round(ph * 824.5 / (2 * math.pi)))
            dpos = pos_counts - pos_counts_prev
            pos_counts_prev = pos_counts
            raw = clip(dpos * 100, -32767, 32767)
            spd_counts = spd_counts + ash(raw - spd_counts, 2)
            # phi10 = pos*4471>>10 后 mod 3600 绕回
            praw = ash(pos_counts * 4471, 10)
            rem = praw - int(praw / 3600) * 3600
            if rem > 1800:
                rem -= 3600
            elif rem < -1800:
                rem += 3600
            phi10 = rem
            phid10 = clip(ash(spd_counts * 4471, 10), -30000, 30000)

        # ---- 被控对象 ----
        u = duty_torque(duty)
        th, thd, ph, phd = rk4(th, thd, ph, phd, u, dt)
        t += dt

        if verbose and int(round(t * 100)) % 5 == 0 and abs(t - round(t, 2)) < dt / 2:
            log.append((t, math.degrees(th), math.degrees(thd), math.degrees(ph),
                        duty, sw.state, sw.e_q, lq.dbg.get('th_tgt', 0), lq.integ,
                        sw.ang_r, sw.spd_q, sw.cos_q_r))

    thd_deg = math.degrees(th)
    if bal_start is not None:
        # 平衡段统计（最后 10 s）
        tail = [s for s in log if s[0] > max(bal_start + 2.0, t_end - 10.0)]
        if tail:
            abs_th = [abs(s[1]) for s in tail]
            abs_th.sort()
            p95 = abs_th[int(0.95 * (len(abs_th) - 1))]
            mean_abs = sum(abs_th) / len(abs_th)
        else:
            p95 = mean_abs = float('nan')
    else:
        p95 = mean_abs = float('nan')
    res = dict(final_theta_deg=thd_deg, lqr_on=lqr_on, sw_state=sw.state,
               bal_time=bal_time, bal_start=bal_start, first_fall=first_fall,
               p95_theta=float(p95), mean_theta=float(mean_abs),
               max_theta_after_bal=max_abs_th_after_bal,
               log=log, states=states)
    return res


def show(name, r):
    ff = r.get('first_fall')
    print("%-26s final=%7.2f°  state=%d  bal_t=%6.2fs  fall@%s  p95|θ|=%6.3f°  mean|θ|=%6.3f°"
          % (name, r['final_theta_deg'], r['sw_state'], r['bal_time'],
             ('%.2fs' % ff) if ff is not None else '  --  ',
             r['p95_theta'], r['mean_theta']))


if __name__ == "__main__":
    print("物理: M11=%.3e Ja=%.3e DET=%.3e U/duty=%.3e N·m  B_arm=%.4f"
          % (M11, Ja, DET, U_PER_DUTY, B_ARM))
    print("能量阈值含义：E=cosθ-1，E=-0.05 <=> 静止在 %.1f°"
          % math.degrees(math.acos(0.95)))
    if "--swp" in sys.argv:
        for inv in (0, 1):
            for est in (-5898, -3000, -2000, -1000):
                r = run(swing_kw=dict(invert_pd=inv, E_STOP_Q=est),
                        t_end=25.0, verbose=False)
                show("inv=%d E_STOP=%d" % (inv, est), r)
    else:
        r = run(t_end=30.0)
        show("默认 (inv=1,E=-2000)", r)
        for s in r['log'][::10]:
            print("t=%5.2f θ=%7.2f θ̇=%8.1f φ=%9.1f duty=%6d st=%d E=%7d tgt=%5d I=%7d"
                  % s)

