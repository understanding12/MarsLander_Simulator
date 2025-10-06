import numpy as np
import matplotlib.pyplot as plt

planets = {
    "Mars": {"g": 3.71, "rho0": 0.02, "H": 11000},
}

planet = "Mars"  # Выбранная планета для посадки
p = planets[planet]  # Параметры выбранной планеты

print(f"=== ОПТИМАЛЬНАЯ ПОСАДКА НА {planet} ===")
print(f"g = {p['g']} м/с², ρ₀ = {p['rho0']} кг/м³, H = {p['H']} м\n")

# ПАРАМЕТРЫ АППАРАТА
dry_mass = 2000  # Сухая масса аппарата (кг)
fuel_mass = 480  # Начальная масса топлива (кг)
mdot = 8  # Максимальный расход топлива (кг/с)
A_body = 15  # Площадь поперечного сечения корпуса (м²)
A_chute = 400  # Площадь парашюта (м²)
Cd_body = 1.2  # Коэффициент сопротивления корпуса
Cd_chute = 1.8  # Коэффициент сопротивления парашюта
T_max = 35000  # Максимальная тяга двигателя (Н)

# НАЧАЛЬНЫЕ УСЛОВИЯ
h = 12000.0  # Начальная высота (м)
v = -290.0  # Начальная скорость (м/с) - отрицательная, т.к. вниз
m = dry_mass + fuel_mass  # Начальная полная масса (кг)
t = 0.0  # Начальное время (с)
dt = 0.1  # Шаг интегрирования (с)

# ПАРАМЕТРЫ ПОСАДКИ
target_speed_final = -0.5  # Целевая скорость при посадке (м/с)
parachute_alt = 11000  # Высота раскрытия парашюта (м)
engine_start_alt = 5000  # Высота включения двигателей (м)
touchdown_alt = 0.0  # Высота поверхности (м)

# ФЛАГИ СОСТОЯНИЯ
parachute = False  # Парашют раскрыт?
engine = False  # Двигатели включены?
crashed = False  # Аварийная посадка?
fuel = fuel_mass  # Текущее количество топлива
final_phase = False  # Финальная фаза посадки?

# ПАРАМЕТРЫ ПИД-РЕГУЛЯТОРА
Kp = 3000.0  # Пропорциональный коэффициент
Ki = 80.0  # Интегральный коэффициент
Kd = 600.0  # Дифференциальный коэффициент
integral = 0.0  # Интегральная составляющая
prev_error = 0.0  # Предыдущая ошибка для дифференцирования
prev_T = 0.0  # Предыдущее значение тяги
max_thrust_rate = 3000  # Максимальная скорость изменения тяги (Н/с)

# МАССИВЫ ДЛЯ ХРАНЕНИЯ ИСТОРИИ
time_hist, h_hist, v_hist, T_hist, a_hist, m_hist, fuel_hist = [], [], [], [], [], [], []

print("🚀 Моделирование запущено...\n")

# ОСНОВНОЙ ЦИКЛ МОДЕЛИРОВАНИЯ
while h > touchdown_alt and t < 1000:
    # Расчет плотности атмосферы (экспоненциальная модель)
    rho = p['rho0'] * np.exp(-h / p['H'])

    # Выбор аэродинамических параметров в зависимости от состояния
    if parachute:
        Cd = Cd_chute  # Коэффициент сопротивления с парашютом
        A = A_chute  # Площадь с парашютом
    else:
        Cd = Cd_body  # Коэффициент сопротивления корпуса
        A = A_body  # Площадь корпуса

    # Расчет силы аэродинамического сопротивления
    # Отрицательный знак - сила направлена против движения
    D = -0.5 * rho * Cd * A * v * abs(v)

    # ЛОГИКА УПРАВЛЕНИЯ ПОСАДКОЙ

    # Раскрытие парашюта при достижении заданной высоты
    if not parachute and h < parachute_alt:
        parachute = True
        print(f"{t:6.1f} c | Парашют раскрыт (h={h:.0f} м, v={abs(v):.1f} м/с)")

    # Включение двигателей после раскрытия парашюта
    if not engine and parachute and h < engine_start_alt:
        engine = True
        print(f"{t:6.1f} c | Двигатели включены! (h={h:.0f} м, v={abs(v):.1f} м/с)")

    # Переход в финальную фазу посадки
    if not final_phase and h < 200:
        final_phase = True
        print(f"{t:6.1f} c | Финальная фаза (h={h:.0f} м)")

    # УПРАВЛЕНИЕ ДВИГАТЕЛЯМИ (ПИД-регулятор)
    T = 0  # Тяга по умолчанию
    if engine and fuel > 0:
        current_speed = abs(v)

        # ВЫБОР ЦЕЛЕВОЙ СКОРОСТИ В ЗАВИСИМОСТИ ОТ ВЫСОТЫ
        # Постепенное уменьшение скорости при снижении
        if h > 2000:
            target_speed = -25
        elif h > 1000:
            target_speed = -15
        elif h > 500:
            target_speed = -8
        elif h > 300:
            target_speed = -5
        elif h > 200:
            target_speed = -3
        elif h > 100:
            target_speed = -1.5
        elif h > 50:
            target_speed = -1.0
        else:
            target_speed = target_speed_final

        # РАСЧЕТ ПИД-РЕГУЛЯТОРА
        error = target_speed - v  # Ошибка управления

        # Интегральная составляющая (с ограничением)
        integral += error * dt * 0.02
        integral = np.clip(integral, -40, 40)

        # Дифференциальная составляющая
        derivative = (error - prev_error) / dt if t > 0 else 0

        # Расчет сырого значения тяги
        T_raw = Kp * error + Ki * integral + Kd * derivative
        T_raw = max(0, min(T_raw, T_max))  # Ограничение по диапазону

        # Ограничение скорости изменения тяги
        max_change = max_thrust_rate * dt
        T = np.clip(T_raw, prev_T - max_change, prev_T + max_change)

        # Сглаживание тяги (фильтр низких частот)
        smoothing_factor = 0.7
        T = prev_T * smoothing_factor + T * (1 - smoothing_factor)

        # Сохранение значений для следующей итерации
        prev_T = T
        prev_error = error

        # РАСХОД ТОПЛИВА
        thrust_ratio = T / T_max
        fuel_consumption = mdot * dt * thrust_ratio

        # Экономия топлива на малых высотах при его нехватке
        if h < 100 and fuel < 30:
            fuel_consumption *= 0.3

        # Проверка наличия топлива
        if fuel_consumption > fuel:
            fuel_consumption = fuel
            T = T * (fuel / fuel_consumption) if fuel_consumption > 0 else 0

        # Обновление количества топлива и массы
        fuel -= fuel_consumption
        m = dry_mass + fuel

        # ВЫВОД ИНФОРМАЦИИ О ПОСАДКЕ
        if engine and t % 5 < dt and h > 10:
            current_g = abs((T + D) / (m * p['g'])) if m > 0 else 0
            print(
                f"{t:6.1f} c | h={h:.0f} м, v={abs(v):.1f} м/с, T={T:.0f} Н, g={current_g:.1f}, топливо={fuel:.0f} кг")
    else:
        # Сброс тяги и состояния регулятора при выключенных двигателях
        T = 0
        prev_T = 0

    # РАСЧЕТ ДИНАМИКИ (УРАВНЕНИЯ ДВИЖЕНИЯ)

    # Суммарная сила: гравитация + тяга + сопротивление
    F_total = -m * p['g'] + T + D

    # Ускорение по второму закону Ньютона
    a = F_total / m

    # ИНТЕГРИРОВАНИЕ УРАВНЕНИЙ ДВИЖЕНИЯ (МЕТОД ЭЙЛЕРА)
    v += a * dt  # Обновление скорости
    h += v * dt  # Обновление высоты
    t += dt  # Обновление времени

    # ПРОВЕРКА КАСАНИЯ ПОВЕРХНОСТИ
    if h <= 0:
        landing_speed = abs(v)
        if landing_speed > 5:
            crashed = True
        break

    # СОХРАНЕНИЕ ИСТОРИИ ДЛЯ ГРАФИКОВ
    time_hist.append(t)
    h_hist.append(h)
    v_hist.append(v)
    T_hist.append(T)
    a_hist.append(a)
    m_hist.append(m)
    fuel_hist.append(fuel)

# ВЫВОД РЕЗУЛЬТАТОВ ПОСАДКИ
print("\n" + "=" * 60)
print("--- РЕЗУЛЬТАТ ---")
print(f"Планета: {planet}")
if h_hist:
    final_speed = abs(v_hist[-1])
else:
    final_speed = abs(v)
print(f"Конечная скорость: {final_speed:.2f} м/с")
print(f"Остаток топлива: {fuel:.1f} кг")
print(f"Общее время: {t:.1f} с")
print(f"Финальная высота: {h:.2f} м")

# ОЦЕНКА КАЧЕСТВА ПОСАДКИ
if h > 0:
    print("❌ Аппарат не достиг поверхности!")
elif crashed:
    print("Посадка аварийная!")
elif final_speed > 3:
    print("Посадка жёсткая, повреждения возможны.")
elif final_speed > 1.5:
    print("Посадка успешная!")
elif final_speed > 0.8:
    print("Посадка мягкая!")
else:
    print("ИДЕАЛЬНАЯ ПОСАДКА!")

# ПОСТРОЕНИЕ ГРАФИКОВ
if time_hist:
    plt.figure(figsize=(15, 10))

    # 1. ГРАФИК ВЫСОТЫ
    plt.subplot(2, 3, 1)
    plt.plot(time_hist, h_hist, 'b-', linewidth=2)
    plt.axhline(y=parachute_alt, color='r', linestyle='--', alpha=0.7, label='Парашют')
    plt.axhline(y=engine_start_alt, color='orange', linestyle='--', alpha=0.7, label='Двигатели')
    plt.axhline(y=200, color='green', linestyle='--', alpha=0.7, label='Финальная фаза')
    plt.ylabel("Высота (м)")
    plt.title(f"Посадка на {planet}")
    plt.legend()
    plt.grid(True)

    # 2. ГРАФИК СКОРОСТИ
    plt.subplot(2, 3, 2)
    plt.plot(time_hist, [abs(v) for v in v_hist], 'orange', linewidth=2)
    plt.axhline(y=abs(target_speed_final), color='red', linestyle='--', alpha=0.7, label='Целевая скорость')
    plt.ylabel("Скорость (м/с)")
    plt.legend()
    plt.grid(True)

    # 3. ГРАФИК ТЯГИ
    plt.subplot(2, 3, 3)
    plt.plot(time_hist, T_hist, 'g-', linewidth=2)
    plt.ylabel("Тяга (Н)")
    plt.xlabel("Время (с)")
    plt.grid(True)

    # 4. ГРАФИК МАССЫ
    plt.subplot(2, 3, 4)
    plt.plot(time_hist, m_hist, 'brown', linewidth=2)
    plt.ylabel("Масса (кг)")
    plt.xlabel("Время (с)")
    plt.grid(True)

    # 5. ГРАФИК ИЗМЕНЕНИЯ ТЯГИ
    plt.subplot(2, 3, 5)
    thrust_changes = [abs(T_hist[i] - T_hist[i - 1]) for i in range(1, len(T_hist))]
    plt.plot(time_hist[1:], thrust_changes, 'red', linewidth=1)
    plt.axhline(y=max_thrust_rate * dt, color='black', linestyle='--', label='Макс. изменение')
    plt.ylabel("Изменение тяги (Н/шаг)")
    plt.xlabel("Время (с)")
    plt.legend()
    plt.grid(True)

    # 6. ГРАФИК ТОПЛИВА
    plt.subplot(2, 3, 6)
    plt.plot(time_hist, fuel_hist, 'purple', linewidth=2)
    plt.ylabel("Топливо (кг)")
    plt.xlabel("Время (с)")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # ДИАГНОСТИКА СИСТЕМЫ
    print(f"\n--- ДИАГНОСТИКА СИСТЕМЫ ---")
    if time_hist:
        max_g = max([abs(a) / p['g'] for a in a_hist])
        min_speed = min([abs(v) for v in v_hist])
        max_speed = max([abs(v) for v in v_hist])

        print(f"Максимальная перегрузка: {max_g:.1f} g")
        print(f"Минимальная скорость: {min_speed:.1f} м/с")
        print(f"Максимальная скорость: {max_speed:.1f} м/с")

        # Анализ работы двигательной установки
        thrust_changes = [abs(T_hist[i] - T_hist[i - 1]) for i in range(1, len(T_hist))]
        max_thrust_change = max(thrust_changes) if thrust_changes else 0
        avg_thrust_change = np.mean(thrust_changes) if thrust_changes else 0
        print(f"Максимальное изменение тяги за шаг: {max_thrust_change:.0f} Н")
        print(f"Среднее изменение тяги за шаг: {avg_thrust_change:.0f} Н")

        engine_on_indices = [i for i, T in enumerate(T_hist) if T > 0]
        if engine_on_indices:
            first_engine_idx = engine_on_indices[0]
            engine_on_height = h_hist[first_engine_idx]
            engine_on_speed = abs(v_hist[first_engine_idx])
            print(f"Двигатели включены на высоте: {engine_on_height:.0f} м")
            print(f"Скорость при включении: {engine_on_speed:.1f} м/с")

            engine_work_time = len(engine_on_indices) * dt
            print(f"Время работы двигателей: {engine_work_time:.1f} с")

            avg_thrust = np.mean([T_hist[i] for i in engine_on_indices])
            print(f"Средняя тяга: {avg_thrust:.0f} Н")

            max_thrust = max(T_hist)
            print(f"Максимальная тяга: {max_thrust:.0f} Н")

            fuel_used = fuel_mass - fuel
            thrust_efficiency = (avg_thrust * engine_work_time) / (fuel_used * 1000) if fuel_used > 0 else 0
            print(f"Использовано топлива: {fuel_used:.1f} кг")
            print(f"Эффективность тяги: {thrust_efficiency:.2f} Н·с/кг")

        # Анализ эффективности парашюта
        if parachute:
            parachute_indices = [i for i, h_val in enumerate(h_hist) if h_val < parachute_alt]
            if parachute_indices:
                parachute_idx = parachute_indices[0]
                if parachute_idx > 0 and engine_on_indices:
                    speed_before_parachute = abs(v_hist[parachute_idx - 1])
                    speed_before_engine = abs(v_hist[engine_on_indices[0] - 1])
                    print(f"Скорость до парашюта: {speed_before_parachute:.1f} м/с")
                    print(f"Скорость до включения двигателей: {speed_before_engine:.1f} м/с")
                    parachute_efficiency = (
                            (speed_before_parachute - speed_before_engine) / speed_before_parachute * 100)
                    print(f"Эффективность парашюта: {parachute_efficiency:.1f}%")

# ОЦЕНКА МИССИИ
print(f"\n--- ОЦЕНКА МИССИИ ---")
if not crashed and final_speed <= 3:

    # Оценка качества посадки по конечной скорости
    if final_speed <= 0.5:
        landing_quality = "ИДЕАЛЬНАЯ"
        landing_score = 30
    elif final_speed <= 1.0:
        landing_quality = "ОТЛИЧНАЯ"
        landing_score = 28
    elif final_speed <= 1.5:
        landing_quality = "ОЧЕНЬ ХОРОШАЯ"
        landing_score = 25
    elif final_speed <= 2.0:
        landing_quality = "ХОРОШАЯ"
        landing_score = 22
    else:
        landing_quality = "УДОВЛЕТВОРИТЕЛЬНАЯ"
        landing_score = 18

    # Расчет общего балла миссии

    print(f"Качество посадки: {landing_quality}")
    print(f"Остаток топлива: {fuel:.1f} кг")

print(f"\n=== СИМУЛЯЦИЯ ЗАВЕРШЕНА ===")