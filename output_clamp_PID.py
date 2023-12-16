import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.prev_error = 0
        self.integral = 0
        
        self.errors = []  # List to store errors
        self.p_terms = []  # List to store p_terms
        self.i_terms = []  # List to store i_terms
        self.d_terms = []  # List to store d_terms
        self.outputs = []  # List to store PID outputs

    def calculate_output(self, setpoint, process_variable):
        error = setpoint - process_variable
        self.errors.append(error)
        print("error: ", error)

        # Proportional term
        p_term = self.kp * error

        
        self.p_terms.append(p_term)
        print("p_term: ", p_term)

        # Integral term
        self.integral += error
        i_term = self.ki * self.integral
        self.i_terms.append(i_term)
        print("i_term: ", i_term)

        # Derivative term
        d_term = self.kd * (error - self.prev_error)
        self.d_terms.append(d_term)

        # PID output
        pid_output = p_term + i_term + d_term

        # Clamp the output within the specified range
        pid_output = max(self.output_min, min(pid_output, self.output_max))

        # Update the previous error for the next iteration
        self.prev_error = error
        print("error 2 : ", error)

        self.outputs.append(pid_output)

        return pid_output

# Example usage
pid_controller = PIDController(kp=0.4, ki=0.1, kd=0.3, output_min=-30, output_max=30)

# Setpoint and process variable values (to be replaced with actual values from your system)
setpoint = 50
process_variable = 0
process_list = []

# Calculate PID output with clamping
num = 0 
while num < 50:
    print("num: ", num)
    output = pid_controller.calculate_output(setpoint, process_variable)
    process_variable = process_variable + output
    process_list.append(process_variable)
    print(f"PID Output: {output}")
    print("process_variable: ", process_variable)
    print("\n")    
    num = num + 1

### Plot the data
plt.figure(figsize=(12, 10))

plt.subplot(3, 2, 1)
plt.plot(pid_controller.errors, label='Error')
plt.axhline(0, color='r', linestyle='--', label='Zero Line')  # Add a horizontal line at y=0
plt.title('Error Over Time')
plt.legend()

plt.subplot(3, 2, 2)
plt.plot(pid_controller.p_terms, label='P Term')
plt.axhline(0, color='r', linestyle='--', label='Zero Line')  # Add a horizontal line at y=0
plt.title('P Term Over Time')
plt.legend()

plt.subplot(3, 2, 3)
plt.plot(pid_controller.i_terms, label='I Term')
plt.axhline(0, color='r', linestyle='--', label='Zero Line')  # Add a horizontal line at y=0
plt.title('I Term Over Time')
plt.legend()

plt.subplot(3, 2, 4)
plt.plot(pid_controller.d_terms, label='D Term')
plt.axhline(0, color='r', linestyle='--', label='Zero Line')  # Add a horizontal line at y=0
plt.title('D Term Over Time')
plt.legend()

plt.subplot(3, 2, 5)
plt.axhline(0, color='r', linestyle='--', label='Zero Line')  # Add a horizontal line at y=0
plt.plot(pid_controller.outputs, label='PID Output', color=(0.4, 0, 1))
plt.title('PID Output Over Time')
plt.legend()

plt.subplot(3, 2, 6)
plt.plot(process_list, label='Process', color=(0.4, 0, 1))
plt.axhline(50, color='r', linestyle='--', label='Zero Line')  # Add a horizontal line at y=0
plt.title('Process Variable Over Time')
plt.legend()

plt.tight_layout()
plt.show()
