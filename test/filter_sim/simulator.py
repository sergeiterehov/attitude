"""
Симулятор IMU-сенсоров для тестирования фильтров ориентации.

Генерирует CSV с истинной траекторией и показаниями сенсоров (акселерометр + гироскоп)
с реалистичным шумом и дрейфом.

Координатная система: ENU (X=East, Y=North, Z=Up)
Частота: 100 Hz (dt = 0.01 с)
"""

import csv
import math
import random
import sys

from scenarios import SCENARIOS, G

DT = 0.01  # 100 Hz
SAMPLE_RATE = 100.0

# === Параметры шумов (BMI160 inside BNO085) ===

# Акселерометр
ACCEL_NOISE_STD = 0.01  # m/s^2 RMS
ACCEL_BIAS = (0.1, -0.05, 0.08)  # m/s^2, постоянный bias по осям
ACCEL_BIAS_DRIFT_SIGMA = 0.001  # m/s^2, медленное блуждание (per step)

# Гироскоп
GYRO_NOISE_STD = 0.0012  # rad/s RMS (~0.07 deg/s)
GYRO_BIAS = (0.003, -0.002, 0.001)  # rad/s (~0.17, -0.11, 0.06 deg/s)
GYRO_BIAS_DRIFT_SIGMA = 0.0001  # rad/s, random walk (per step)


def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """
    Матрица поворота из world (ENU) в body frame.
    Body frame: X=forward, Y=left, Z=up (авиационное соглашение).
    Порядок: ZYX (yaw-pitch-roll).
    """
    phi = math.radians(roll_deg)  # roll
    theta = math.radians(pitch_deg)  # pitch
    psi = math.radians(yaw_deg)  # yaw

    cphi = math.cos(phi)
    sphi = math.sin(phi)
    ctheta = math.cos(theta)
    stheta = math.sin(theta)
    cpsi = math.cos(psi)
    spsi = math.sin(psi)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    # Транспонируем для world→body
    r00 = cpsi * ctheta
    r01 = spsi * ctheta
    r02 = -stheta
    r10 = cpsi * stheta * sphi - spsi * cphi
    r11 = spsi * stheta * sphi + cpsi * cphi
    r12 = ctheta * sphi
    r20 = cpsi * stheta * cphi + spsi * sphi
    r21 = spsi * stheta * cphi - cpsi * sphi
    r22 = ctheta * cphi

    return (
        (r00, r01, r02),
        (r10, r11, r12),
        (r20, r21, r22),
    )


def mat_vec_mul(R, v):
    """Умножение матрицы 3x3 на вектор."""
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def euler_rates_to_body_rates(roll_deg, pitch_deg, yaw_deg, d_roll, d_pitch, d_yaw):
    """
    Преобразование производных Эйлера (deg/s) в угловые скорости body frame (rad/s).

    p = φ̇ - ψ̇ sin(θ)
    q = θ̇ cos(φ) + ψ̇ cos(θ) sin(φ)
    r = -θ̇ sin(φ) + ψ̇ cos(θ) cos(φ)
    """
    phi = math.radians(roll_deg)
    theta = math.radians(pitch_deg)

    # d_roll, d_pitch, d_yaw — в rad/s
    phi_dot = d_roll
    theta_dot = d_pitch
    psi_dot = d_yaw

    p = phi_dot - psi_dot * math.sin(theta)
    q = theta_dot * math.cos(phi) + psi_dot * math.cos(theta) * math.sin(phi)
    r = -theta_dot * math.sin(phi) + psi_dot * math.cos(theta) * math.cos(phi)

    return p, q, r


def compute_accel_body(roll_deg, pitch_deg, yaw_deg, a_linear_world):
    """
    Вычислить показания акселерометра в body frame.

    Акелерометр измеряет specific force: a_specific = a_linear - g
    В ENU: g_world = (0, 0, -G)
    a_specific_world = a_linear_world + (0, 0, G)

    Затем трансформируем в body frame.
    """
    # Specific force в world frame (реакция опоры)
    a_specific_world = (
        a_linear_world[0],
        a_linear_world[1],
        a_linear_world[2] + G,
    )

    # Трансформация в body frame
    R = euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
    a_body = mat_vec_mul(R, a_specific_world)

    return a_body


def compute_linear_accel_world(state_prev, state_curr, dt):
    """
    Вычислить линейное ускорение в world frame из изменения скорости.
    Возвращает (ax, ay, az) в m/s^2.
    """
    # Для простоты: считаем, что скорость направлена вдоль heading
    # v_ground — горизонтальная скорость, v_vertical — вертикальная

    # Разложение скорости в world frame
    heading_prev = math.radians(state_prev.yaw)
    heading_curr = math.radians(state_curr.yaw)

    vx_prev = state_prev.v_ground * math.cos(heading_prev)  # East
    vy_prev = state_prev.v_ground * math.sin(heading_prev)  # North
    vz_prev = state_prev.v_vertical

    vx_curr = state_curr.v_ground * math.cos(heading_curr)
    vy_curr = state_curr.v_ground * math.sin(heading_curr)
    vz_curr = state_curr.v_vertical

    ax = (vx_curr - vx_prev) / dt
    ay = (vy_curr - vy_prev) / dt
    az = (vz_curr - vz_prev) / dt

    return ax, ay, az


def compute_angular_rates(state_prev, state_curr, dt):
    """
    Вычислить производные Эйлера (deg/s) из изменения углов.
    """
    d_roll = (state_curr.roll - state_prev.roll) / dt  # deg/s
    d_pitch = (state_curr.pitch - state_prev.pitch) / dt
    d_yaw = (state_curr.yaw - state_prev.yaw) / dt

    return d_roll, d_pitch, d_yaw


def add_noise_accel(true_ax, true_ay, true_az, accel_bias_drift):
    """Добавить шум и дрейф к показаниям акселерометра."""
    noise = (
        random.gauss(0, ACCEL_NOISE_STD),
        random.gauss(0, ACCEL_NOISE_STD),
        random.gauss(0, ACCEL_NOISE_STD),
    )
    # Обновляем дрейф bias (random walk)
    accel_bias_drift[0] += random.gauss(0, ACCEL_BIAS_DRIFT_SIGMA)
    accel_bias_drift[1] += random.gauss(0, ACCEL_BIAS_DRIFT_SIGMA)
    accel_bias_drift[2] += random.gauss(0, ACCEL_BIAS_DRIFT_SIGMA)

    return (
        true_ax + ACCEL_BIAS[0] + accel_bias_drift[0] + noise[0],
        true_ay + ACCEL_BIAS[1] + accel_bias_drift[1] + noise[1],
        true_az + ACCEL_BIAS[2] + accel_bias_drift[2] + noise[2],
    )


def add_noise_gyro(true_gx, true_gy, true_gz, gyro_bias_drift):
    """Добавить шум и дрейф к показаниям гироскопа."""
    noise = (
        random.gauss(0, GYRO_NOISE_STD),
        random.gauss(0, GYRO_NOISE_STD),
        random.gauss(0, GYRO_NOISE_STD),
    )
    gyro_bias_drift[0] += random.gauss(0, GYRO_BIAS_DRIFT_SIGMA)
    gyro_bias_drift[1] += random.gauss(0, GYRO_BIAS_DRIFT_SIGMA)
    gyro_bias_drift[2] += random.gauss(0, GYRO_BIAS_DRIFT_SIGMA)

    return (
        true_gx + GYRO_BIAS[0] + gyro_bias_drift[0] + noise[0],
        true_gy + GYRO_BIAS[1] + gyro_bias_drift[1] + noise[1],
        true_gz + GYRO_BIAS[2] + gyro_bias_drift[2] + noise[2],
    )


def simulate_scenario(scenario_def):
    """
    Симулировать один сценарий.
    Возвращает список строк CSV (без header).
    """
    name = scenario_def["name"]
    maneuvers = scenario_def["maneuvers"]

    rows = []

    # Начальное состояние
    class SimpleState:
        pass

    prev_state = SimpleState()
    prev_state.roll = 0.0
    prev_state.pitch = 0.0
    prev_state.yaw = 0.0
    prev_state.v_ground = 60.0
    prev_state.v_vertical = 0.0

    # Дрейфы bias (mutable lists)
    accel_bias_drift = [0.0, 0.0, 0.0]
    gyro_bias_drift = [0.0, 0.0, 0.0]

    t_global = 0.0  # глобальное время (сек)

    for maneuver in maneuvers:
        # Состояние на входе в манёвр
        state_in = SimpleState()
        state_in.roll = prev_state.roll
        state_in.pitch = prev_state.pitch
        state_in.yaw = prev_state.yaw
        state_in.v_ground = prev_state.v_ground
        state_in.v_vertical = prev_state.v_vertical

        n_steps = int(maneuver.duration / DT)

        for i in range(n_steps):
            t_local = i * DT

            # Текущее истинное состояние
            curr_state = maneuver.compute_state(t_local, state_in)

            # Истинный акселерометр в body frame (из манёвра)
            true_ax, true_ay, true_az = maneuver.compute_accel_body(t_local, state_in)

            # Производные Эйлера → угловые скорости body frame
            d_roll, d_pitch, d_yaw = compute_angular_rates(prev_state, curr_state, DT)
            d_roll_rad = math.radians(d_roll)
            d_pitch_rad = math.radians(d_pitch)
            d_yaw_rad = math.radians(d_yaw)

            true_gx, true_gy, true_gz = euler_rates_to_body_rates(
                curr_state.roll, curr_state.pitch, curr_state.yaw,
                d_roll_rad, d_pitch_rad, d_yaw_rad,
            )

            # Шум
            sim_ax, sim_ay, sim_az = add_noise_accel(true_ax, true_ay, true_az, accel_bias_drift)
            sim_gx, sim_gy, sim_gz = add_noise_gyro(true_gx, true_gy, true_gz, gyro_bias_drift)

            t_ms = int(t_global * 1000)

            rows.append(
                f"{name},{t_ms},"
                f"{curr_state.roll:.3f},{curr_state.pitch:.3f},{curr_state.yaw:.3f},"
                f"{curr_state.v_ground:.3f},{curr_state.v_vertical:.3f},"
                f"{sim_ax:.6f},{sim_ay:.6f},{sim_az:.6f},"
                f"{sim_gx:.6f},{sim_gy:.6f},{sim_gz:.6f}"
            )

            # Обновляем prev_state
            prev_state.roll = curr_state.roll
            prev_state.pitch = curr_state.pitch
            prev_state.yaw = curr_state.yaw
            prev_state.v_ground = curr_state.v_ground
            prev_state.v_vertical = curr_state.v_vertical

            t_global += DT

    return rows


def main():
    random.seed(42)  # воспроизводимость

    # Header
    print(
        "scenario,t_ms,"
        "true_roll,true_pitch,true_yaw,"
        "v_ground,v_vertical,"
        "ax,ay,az,"
        "gx,gy,gz"
    )

    for scenario_def in SCENARIOS:
        rows = simulate_scenario(scenario_def)
        for row in rows:
            print(row)


if __name__ == "__main__":
    main()
