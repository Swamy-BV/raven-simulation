# SPDX-License-Identifier: AGPL-3.0-only
"""Deterministic yaw isolation inputs; no perception or scene ground truth."""
import math


def yaw_response_sample(elapsed, origin, heading):
    """Return phase, position, velocity, acceleration, yaw and yaw rate in NED."""
    zero = [0.0, 0.0, 0.0]
    phase = elapsed
    hold = [("heading_hold", 5, 0), ("yaw_step_positive", 8, 20),
            ("yaw_step_negative", 8, -20), ("yaw_step_return", 8, 0)]
    for name, duration, degrees in hold:
        if phase < duration:
            return name, list(origin), zero, zero, heading + math.radians(degrees), 0.0
        phase -= duration
    for name, feedforward in [("yaw_sine_no_feedforward", False),
                              ("yaw_sine_with_feedforward", True)]:
        duration = 16.0
        if phase < duration:
            frequency = math.tau / 8
            angle = math.radians(10) * math.sin(frequency * phase)
            rate = math.radians(10) * frequency * math.cos(frequency * phase)
            return name, list(origin), zero, zero, heading + angle, rate if feedforward else 0.0
        phase -= duration
    # Sideways translation at fixed yaw tests bank/yaw coupling, using the same
    # mixed XY velocity + Z position interface as Guidance. Smooth ramps have
    # matching velocity and acceleration feed-forward, without vision feedback.
    ramp, cruise, speed = 6.0, 4.0, 18.0
    distance = speed * (ramp + cruise)
    if phase < 2 * ramp + cruise:
        if phase < ramp:
            velocity = speed / 2 * (1 - math.cos(math.pi * phase / ramp))
            acceleration = speed * math.pi / (2 * ramp) * math.sin(math.pi * phase / ramp)
            position = speed / 2 * (phase - ramp / math.pi * math.sin(math.pi * phase / ramp))
        elif phase < ramp + cruise:
            velocity, acceleration = speed, 0.0
            position = speed * ramp / 2 + speed * (phase - ramp)
        else:
            t = phase - ramp - cruise
            velocity = speed / 2 * (1 + math.cos(math.pi * t / ramp))
            acceleration = -speed * math.pi / (2 * ramp) * math.sin(math.pi * t / ramp)
            position = speed * ramp / 2 + speed * cruise + speed / 2 * (t + ramp / math.pi * math.sin(math.pi * t / ramp))
        return "straight_fixed_yaw", [origin[0] + position, origin[1], origin[2]], [velocity, 0.0, 0.0], [acceleration, 0.0, 0.0], heading, 0.0
    phase -= 2 * ramp + cruise
    base = [origin[0] + distance, origin[1], origin[2]]
    if phase < 5:
        return "straight_settle", base, zero, zero, heading, 0.0
    phase -= 5
    radius, speed, ramp = 80.0, 18.0, 8.0
    maximum_rate = speed / radius
    cruise = math.tau / maximum_rate - ramp
    duration = 2 * ramp + cruise
    for name, tangent in [("circle_fixed_yaw", False), ("circle_tangent_yaw", True)]:
        if phase < duration:
            if phase < ramp:
                omega = maximum_rate / 2 * (1 - math.cos(math.pi * phase / ramp))
                alpha = maximum_rate * math.pi / (2 * ramp) * math.sin(math.pi * phase / ramp)
                angle = maximum_rate / 2 * (phase - ramp / math.pi * math.sin(math.pi * phase / ramp))
            elif phase < ramp + cruise:
                omega, alpha = maximum_rate, 0.0
                angle = maximum_rate * ramp / 2 + maximum_rate * (phase - ramp)
            else:
                t = phase - ramp - cruise
                omega = maximum_rate / 2 * (1 + math.cos(math.pi * t / ramp))
                alpha = -maximum_rate * math.pi / (2 * ramp) * math.sin(math.pi * t / ramp)
                angle = maximum_rate * ramp / 2 + maximum_rate * cruise + maximum_rate / 2 * (t + ramp / math.pi * math.sin(math.pi * t / ramp))
            sine, cosine = math.sin(angle), math.cos(angle)
            position = [base[0] + radius * (1 - cosine), base[1] + radius * sine, base[2]]
            velocity = [radius * sine * omega, radius * cosine * omega, 0.0]
            acceleration = [radius * (cosine * omega * omega + sine * alpha), radius * (-sine * omega * omega + cosine * alpha), 0.0]
            return name, position, velocity, acceleration, heading - angle if tangent else heading, -omega if tangent else 0.0
        phase -= duration
        if phase < 5:
            return name + "_settle", base, zero, zero, heading, 0.0
        phase -= 5
    return None
