import matplotlib.pyplot as plt
import time
from pyvesc.VESC import VESC
import numpy as np

# Initialize parameters
duty_cycle = 0.95
acceleration_delay = 0.5
idle_current = None
magnet_pole_pairs = None

# Data storage
currents = []
erpms = []
efficiencies = []
voltages = []

# Initialize VESC connection
try:
    vesc = VESC(serial_port='/dev/ttyACM0')
    print("VESC connection established.")
except Exception as e:
    print(f"Error: Unable to connect to VESC: {e}")
    exit(1)

# Ask for the number of magnet pole pairs
try:
    magnet_pole_pairs = int(input("Enter the number of magnet pole pairs for your motor: "))
except ValueError:
    print("Invalid input. Please enter a valid integer.")
    exit(1)

# Measure idle current
try:
    print("Measuring idle current...")
    state = vesc.get_measurements()
    idle_current = state.avg_input_current if state else 0
    print(f"Idle current measured: {idle_current:.2f} A")
except Exception as e:
    print(f"Error measuring idle current: {e}")
    exit(1)

# Simulate starting the motor
print(f"Starting motor with {duty_cycle * 100}% duty cycle...")
current_duty_cycle = 0.1
while current_duty_cycle < duty_cycle:
    print(f"Accelerating... Current duty cycle: {current_duty_cycle * 100:.1f}%")
    try:
        vesc.set_duty_cycle(current_duty_cycle)
        current_duty_cycle += 0.05
        time.sleep(acceleration_delay)
    except Exception as e:
        print(f"Error setting duty cycle: {e}")
        exit(1)

print("Motor running. Waiting for stable ERPM...")
time.sleep(2)  # Simulate delay for motor to stabilize

# Wait for user input to start measurement
print("Press Enter to start the measurement...")
input()  # Wait for the user to press Enter
print("Measurement started.")

# Measurement loop
while True:
    try:
        # Retry mechanism to handle timeouts
        retries = 3
        state = None
        for _ in range(retries):
            state = vesc.get_measurements()
            if state:
                break
            time.sleep(0.1)  # Short delay between retries

        if state is None:
            print("Error: No data received from VESC after multiple attempts. Stopping measurements.")
            break

        # Use avg_input_current instead of avg_motor_current
        current = state.avg_input_current if state else 0
        erpm = state.rpm if state else 0
        voltage = state.v_in if state else 0

        # Calculate RPM from ERPM and magnet pole pairs
        rpm = erpm / magnet_pole_pairs if magnet_pole_pairs > 0 else 0

        # Log values continuously
        currents.append(current)
        erpms.append(rpm)
        voltages.append(voltage)

        print(f"Logging data - Battery Current: {current:.2f} A, RPM: {rpm:.2f}, Voltage: {voltage:.2f} V")

        # Stop if current > 60A or rpm < (1000 / magnet_pole_pairs)
        if current > 60 or rpm < 1000 / magnet_pole_pairs:
            print("Battery current > 60A or RPM below threshold. Stopping measurements.")
            break

        time.sleep(0.1)  # Simulate measurement delay

    except Exception as e:
        print(f"Error during measurement: {e}")
        break

# Fit parameters for efficiency calculation using the first 10 data points
print("Fitting parameters for efficiency calculation using the first 10 data points...")
if len(currents) >= 10:
    # Use the first 10 points to calculate R and c
    i_values = np.array(currents[:10])
    v_values = np.array(voltages[:10])
    rpm_values = np.array(erpms[:10])

    # Set up the system of equations v = A*R + c*2*pi*u
    A = np.vstack([i_values, 2 * np.pi * rpm_values]).T
    coefficients, _, _, _ = np.linalg.lstsq(A, v_values, rcond=None)
    R, c = coefficients

    print(f"Calculated parameters: R = {R:.4f} Ohms, c = {c:.4f}")

    # Calculate efficiency based on the fitted parameters
    efficiencies = []
    for i in range(len(currents)):
        input_power = voltages[i] * currents[i]  # Input power in Watts
        estimated_voltage = R * currents[i] + c * 2 * np.pi * erpms[i]
        mechanical_power = max(0, estimated_voltage * currents[i])  # Mechanical power
        efficiency = (mechanical_power / input_power) * 100 if input_power > 0 else 0
        efficiencies.append(max(0, min(100, efficiency)))  # Constrain efficiency to realistic range
else:
    print("Not enough data points for parameter fitting. At least 10 are required.")

# Stop the motor
try:
    vesc.set_duty_cycle(0)
    print("Motor stopped.")
except Exception as e:
    print(f"Error stopping motor: {e}")

# Plot results
try:
    plt.figure(figsize=(10, 6))
    plt.plot(currents, efficiencies, label="Efficiency Curve")
    plt.title("Motor Efficiency vs Battery Current")
    plt.xlabel("Battery Current (A)")
    plt.ylabel("Efficiency (%)")
    plt.xlim(0, 60)
    plt.ylim(0, 100)
    plt.grid(True)
    plt.legend()
    plt.savefig("Wirkungsgradkurve.jpg")
    plt.show()
    print("Efficiency curve saved as 'Wirkungsgradkurve.jpg'.")

except Exception as e:
    print(f"Error during plotting: {e}")

# Close VESC connection
try:
    del vesc  # Ensure proper cleanup if close() is not available
    print("VESC connection closed.")
except Exception as e:
    print(f"Error closing VESC connection: {e}")
