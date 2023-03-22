class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral_error = 0

    def update(self, current_value):
        error = self.setpoint - current_value

        # componente proporcional
        p = self.kp * error

        # componente integral
        self.integral_error += error
        i = self.ki * self.integral_error

        # componente derivativo
        derivative_error = error - self.last_error
        d = self.kd * derivative_error
        self.last_error = error

        # sinal de controle
        output = p + i + d

        return output
