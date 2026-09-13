"""
Определения сценариев для симуляции полёта.

Каждый сценарий — последовательность манёвров с физически согласованными параметрами.
Координатная система: ENU (X=East, Y=North, Z=Up).
"""

import math


def smoothstep(t, t_start, t_end, value_start, value_end):
    """Плавная интерполяция (smoothstep) для переходных процессов."""
    if t <= t_start:
        return value_start
    if t >= t_end:
        return value_end
    x = (t - t_start) / (t_end - t_start)
    x = x * x * (3.0 - 2.0 * x)
    return value_start + (value_end - value_start) * x


G = 9.81  # m/s^2


class State:
    """Состояние ЛА в данный момент времени."""

    def __init__(self):
        self.roll = 0.0  # deg
        self.pitch = 0.0  # deg
        self.yaw = 0.0  # deg
        self.v_ground = 0.0  # m/s (горизонтальная скорость)
        self.v_vertical = 0.0  # m/s (вертикальная скорость)


class Maneuver:
    """Базовый класс манёвра."""

    def __init__(self, name, duration):
        self.name = name
        self.duration = duration  # seconds

    def compute_state(self, t_local, state_in):
        """
        Вычислить состояние в момент t_local (секунды от начала манёвра).
        state_in — состояние на входе в манёвр.
        Возвращает State.
        """
        raise NotImplementedError

    def compute_accel_body(self, t_local, state_in):
        """
        Вычислить показания акселерометра в body frame (specific force).
        По умолчанию: (0, 0, g) — покой.
        Переопределяется в подклассах для разных манёвров.
        """
        return (0.0, 0.0, G)


class LevelFlight(Maneuver):
    """Горизонтальный полёт с постоянной скоростью."""

    def __init__(self, v_ground=60.0, duration=30.0):
        super().__init__("level_flight", duration)
        self.v_ground = v_ground

    def compute_state(self, t_local, state_in):
        s = State()
        s.roll = 0.0
        s.pitch = 0.0
        s.yaw = state_in.yaw
        s.v_ground = self.v_ground
        s.v_vertical = 0.0
        return s


class Braking(Maneuver):
    """Торможение с постоянной перегрузкой."""

    def __init__(self, decel_g=0.3, duration=5.0):
        super().__init__("braking", duration)
        self.decel_g = decel_g
        self.transition_time = 1.5  # плавный ввод за 1.5 сек

    def compute_state(self, t_local, state_in):
        s = State()
        # Торможение плавное
        decel = smoothstep(t_local, 0, self.transition_time, 0, self.decel_g * G)
        v = state_in.v_ground - decel * t_local
        v = max(v, 0)

        s.roll = 0.0
        s.pitch = 0.0
        s.yaw = state_in.yaw
        s.v_ground = v
        s.v_vertical = 0.0
        return s

    def compute_accel_body(self, t_local, state_in):
        """
        При торможении акселерометр видит продольное ускорение (положительное X) + gravity.
        При торможении пилота бросает вперёд — акселерометр измеряет положительное X.
        a_body = (a_brake, 0, g)
        """
        decel = smoothstep(t_local, 0, self.transition_time, 0, self.decel_g * G)
        return (decel, 0.0, G)


class Acceleration(Maneuver):
    """Разгон с постоянной перегрузкой."""

    def __init__(self, accel_g=0.3, v_start=40.0, duration=5.0):
        super().__init__("acceleration", duration)
        self.accel_g = accel_g
        self.v_start = v_start
        self.transition_time = 1.5

    def compute_state(self, t_local, state_in):
        s = State()
        accel = smoothstep(t_local, 0, self.transition_time, 0, self.accel_g * G)
        v = state_in.v_ground + accel * t_local

        s.roll = 0.0
        s.pitch = 0.0
        s.yaw = state_in.yaw
        s.v_ground = v
        s.v_vertical = 0.0
        return s

    def compute_accel_body(self, t_local, state_in):
        """
        При разгоне акселерометр видит продольное ускорение (отрицательное X) + gravity.
        При разгоне пилота вдавливает в кресло — акселерометр измеряет отрицательное X.
        a_body = (-a_accel, 0, g)
        """
        accel = smoothstep(t_local, 0, self.transition_time, 0, self.accel_g * G)
        return (-accel, 0.0, G)


class CoordinatedTurn(Maneuver):
    """
    Координированный поворот с постоянным креном.
    Физика: turn_rate = g * tan(bank) / v_ground
    """

    def __init__(self, bank_deg=30.0, direction=1, v_ground=60.0, duration=120.0):
        super().__init__("coordinated_turn", duration)
        self.bank_deg = bank_deg
        self.direction = direction  # +1 = left, -1 = right
        self.v_ground = v_ground
        self.transition_time = 3.0  # ввод в крен за 3 сек

    def compute_state(self, t_local, state_in):
        s = State()

        # Плавный ввод крена
        bank = smoothstep(t_local, 0, self.transition_time, 0, self.bank_deg * self.direction)

        # Turn rate из физики координированного поворота
        bank_rad = math.radians(bank)
        turn_rate = G * math.tan(bank_rad) / self.v_ground  # rad/s

        s.roll = bank
        s.pitch = 0.0
        s.yaw = state_in.yaw + math.degrees(turn_rate) * t_local
        s.v_ground = self.v_ground
        s.v_vertical = 0.0
        return s

    def compute_accel_body(self, t_local, state_in):
        """
        В координированном повороте акселерометр измеряет реакцию на подъёмную силу.
        a_body = (0, 0, g/cos(bank)) — перегрузка вдоль body Z.
        """
        bank = smoothstep(t_local, 0, self.transition_time, 0, self.bank_deg * self.direction)
        bank_rad = math.radians(bank)
        # Перегрузка вдоль body Z
        a_z = G / math.cos(bank_rad) if abs(bank_rad) < math.pi / 2 else G
        return (0.0, 0.0, a_z)


class CarTurn(Maneuver):
    """
    Автомобильный поворот (без крена).
    Боковое ускорение создаёт центробежную силу.
    """

    def __init__(self, lateral_g=0.3, v_ground=17.0, direction=1, duration=10.0):
        super().__init__("car_turn", duration)
        self.lateral_g = lateral_g
        self.v_ground = v_ground
        self.direction = direction
        self.transition_time = 2.0

    def compute_state(self, t_local, state_in):
        s = State()

        # Плавный ввод поворота
        lateral = smoothstep(t_local, 0, self.transition_time, 0, self.lateral_g * G)

        # Turn rate из бокового ускорения: omega = a_lateral / v
        turn_rate = lateral / self.v_ground  # rad/s

        s.roll = 0.0  # машина не кренится (или минимально)
        s.pitch = 0.0
        s.yaw = state_in.yaw + math.degrees(turn_rate) * t_local * self.direction
        s.v_ground = self.v_ground
        s.v_vertical = 0.0
        return s

    def compute_accel_body(self, t_local, state_in):
        """
        В автомобильном повороте акселерометр видит боковое ускорение + gravity.
        a_body = (0, a_lateral, g) — центробежное ускорение вдоль body Y.
        """
        lateral = smoothstep(t_local, 0, self.transition_time, 0, self.lateral_g * G)
        # Центробежное ускорение направлено наружу от центра поворота
        # Если direction=1 (левый поворот), центробежная сила вправо (отрицательное Y)
        a_y = -lateral * self.direction
        return (0.0, a_y, G)


class Climb(Maneuver):
    """Набор высоты с постоянным тангажом."""

    def __init__(self, pitch_deg=3.0, v_ground=60.0, duration=30.0):
        super().__init__("climb", duration)
        self.pitch_deg = pitch_deg
        self.v_ground = v_ground
        self.transition_time = 3.0

    def compute_state(self, t_local, state_in):
        s = State()

        pitch = smoothstep(t_local, 0, self.transition_time, 0, self.pitch_deg)
        pitch_rad = math.radians(pitch)

        s.roll = 0.0
        s.pitch = pitch
        s.yaw = state_in.yaw
        s.v_ground = self.v_ground
        s.v_vertical = math.sin(pitch_rad) * self.v_ground
        return s

    def compute_accel_body(self, t_local, state_in):
        """
        В наборе высоты акселерометр видит примерно (0, 0, g) — lift балансирует gravity.
        """
        return (0.0, 0.0, G)


class UrbanCycle(Maneuver):
    """
    Городской цикл: последовательность манёвров внутри одного сценария.
    Это мета-манёвр, который содержит подсценарии.
    """

    def __init__(self, v_ground=17.0):
        self.v_ground = v_ground
        self.sub_maneuvers = [
            LevelFlight(v_ground, duration=10),
            Braking(decel_g=0.3, duration=5),
            LevelFlight(v_ground=10, duration=5),
            Acceleration(accel_g=0.3, v_start=10, duration=5),
            LevelFlight(v_ground=17, duration=10),
            CarTurn(lateral_g=0.3, v_ground=17, direction=1, duration=10),
            LevelFlight(v_ground=17, duration=10),
            CarTurn(lateral_g=0.2, v_ground=17, direction=-1, duration=8),
            LevelFlight(v_ground=17, duration=10),
            Braking(decel_g=0.2, duration=5),
            LevelFlight(v_ground=5, duration=10),
            Acceleration(accel_g=0.2, v_start=5, duration=5),
            LevelFlight(v_ground=17, duration=10),
            CarTurn(lateral_g=0.4, v_ground=17, direction=1, duration=8),
            LevelFlight(v_ground=17, duration=9),
        ]
        total_duration = sum(m.duration for m in self.sub_maneuvers)
        super().__init__("urban_cycle", total_duration)

    def compute_state(self, t_local, state_in):
        """Прогоняем подсценарии последовательно."""
        t_remaining = t_local
        current_state = State()
        current_state.v_ground = state_in.v_ground
        current_state.yaw = state_in.yaw

        for sub in self.sub_maneuvers:
            if t_remaining <= sub.duration:
                return sub.compute_state(t_remaining, current_state)
            # Обновляем состояние после завершения подсценария
            current_state = sub.compute_state(sub.duration, current_state)
            t_remaining -= sub.duration

        # Если вышли за пределы — возвращаем последнее состояние
        return current_state

    def compute_accel_body(self, t_local, state_in):
        """Делегируем подсценариям."""
        t_remaining = t_local
        current_state = State()
        current_state.v_ground = state_in.v_ground
        current_state.yaw = state_in.yaw

        for sub in self.sub_maneuvers:
            if t_remaining <= sub.duration:
                return sub.compute_accel_body(t_remaining, current_state)
            current_state = sub.compute_state(sub.duration, current_state)
            t_remaining -= sub.duration

        return (0.0, 0.0, G)


# === Готовые сценарии ===

SCENARIOS = [
    {
        "name": "straight_flight",
        "maneuvers": [LevelFlight(v_ground=60.0, duration=30.0)],
    },
    {
        "name": "braking",
        "maneuvers": [
            LevelFlight(v_ground=60.0, duration=5.0),  # установившийся
            Braking(decel_g=0.3, duration=5.0),
            LevelFlight(v_ground=45.0, duration=5.0),  # после торможения
        ],
    },
    {
        "name": "acceleration",
        "maneuvers": [
            LevelFlight(v_ground=40.0, duration=5.0),
            Acceleration(accel_g=0.3, v_start=40.0, duration=5.0),
            LevelFlight(v_ground=55.0, duration=5.0),
        ],
    },
    {
        "name": "car_turn",
        "maneuvers": [
            LevelFlight(v_ground=17.0, duration=5.0),
            CarTurn(lateral_g=0.3, v_ground=17.0, direction=1, duration=10.0),
            LevelFlight(v_ground=17.0, duration=5.0),
        ],
    },
    {
        "name": "coordinated_turn_30",
        "maneuvers": [
            LevelFlight(v_ground=60.0, duration=5.0),
            CoordinatedTurn(bank_deg=30.0, direction=1, v_ground=60.0, duration=120.0),
            LevelFlight(v_ground=60.0, duration=5.0),
        ],
    },
    {
        "name": "urban_cycle",
        "maneuvers": [UrbanCycle(v_ground=17.0)],
    },
]
