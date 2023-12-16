class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max, anti_windup_min, anti_windup_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max
        
        self.anti_windup_min = anti_windup_min
        self.anti_windup_max = anti_windup_max
        
        self.prev_error = 0
        self.integral = 0

    def calculate_output(self, setpoint, process_variable):
        error = setpoint - process_variable

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error
        # Apply anti-windup to limit the integral term
        self.integral = max(self.anti_windup_min, min(self.anti_windup_max, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * (error - self.prev_error)

        # PID output
        pid_output = p_term + i_term + d_term

        # Clamp the output within the specified range
        pid_output = max(self.output_min, min(pid_output, self.output_max))

        # Update the previous error for the next iteration
        self.prev_error = error

        return pid_output

# Example usage
pid_controller = PIDController(kp=0.5, ki=0.1, kd=0.2, output_min=-10, output_max=10, anti_windup_min=-5, anti_windup_max=5)

# Setpoint and process variable values (to be replaced with actual values from your system)
setpoint = 50
process_variable = 45

# Calculate PID output with anti-windup and clamping
output = pid_controller.calculate_output(setpoint, process_variable)

print(f"PID Output: {output}")

