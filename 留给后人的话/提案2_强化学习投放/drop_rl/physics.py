"""SI units; horizontal linear drag and vertical vacuum fall to the rim plane."""
import math

G = 9.80665


def fall_time(height, vertical_velocity=0.0):
    """z up; positive vertical velocity means rising; target rim is z=0."""
    if not all(math.isfinite(v) for v in (height, vertical_velocity)) or height < 0:
        raise ValueError('invalid fall initial condition')
    return (vertical_velocity + math.sqrt(vertical_velocity**2 + 2 * G * height)) / G


def drift(velocity, wind, drag, duration):
    if drag < 0 or duration < 0:
        raise ValueError('drag and duration must be nonnegative')
    if drag < 1e-10:
        return velocity * duration
    return wind * duration + (velocity - wind) * (-math.expm1(-drag * duration)) / drag


def ballistic_landing(position, velocity, height, wind=(0.0, 0.0), drag=0.0):
    t = fall_time(height)
    return tuple(position[i] + drift(velocity[i], wind[i], drag, t) for i in range(2)), t


def first_order_axis(position, velocity, command, disturbance, tau, dt):
    """Exact velocity-response integration; avoids Euler dt/tau instability."""
    steady = command + tau * disturbance
    decay = math.exp(-dt / tau)
    return (position + steady * dt + (velocity - steady) * tau * (1 - decay),
            steady + (velocity - steady) * decay)


def clamp_vector(x, y, limit):
    scale = min(1.0, limit / max(math.hypot(x, y), 1e-12))
    return x * scale, y * scale
