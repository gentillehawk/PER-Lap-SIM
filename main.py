import math
import matplotlib.pyplot as plt
import scipy.interpolate as interp

def accelThrust(torque):
    """Calculates acceleration force of the drivetrain based on torque.
    Takes into account gear ratio, drivetrain losses, tire radius, and
    motor controller efficiency.
    Input: Torque (N-m)
    Output: Force (N)
    """
    return (torque * gearRatio * driveEfficiency) / tireRadius

def deccelRate(speed):
    """Calculates braking acceleration of the car.
    Takes into account aero drag and rolling resistance.
    Input: Speed (m/s)
    Output: Acceleration (m/s^2)
    """
    brakeAccel = -(maxBrakingGs * 9.81)
    drag = Cd * rho * (speed ** 2) * Af / 2
    rollingResistance = Cr * (massCar + massDriver) * 9.81
    netBrakingForce = drag + rollingResistance
    return brakeAccel - (netBrakingForce / (massCar + massDriver))

def brakingDistance(curSpeed, desSpeed):
    """Calculates current braking distance of the car from current speed to
    desired final speed. Does not take into account aero drag or rolling
    resistance.
    Input: Current Speed (m/s)
           Desired Speed (m/s)
    Output: Distance (m)
    """
    brakeDist = ((curSpeed ** 2 - desSpeed ** 2) / (2 * maxBrakingGs * 9.81))
    return brakeDist

def accelRate(accelThrust, speed):
    """Calculates acceleration of the car based on acceleration forces.
    Takes into account aero drag and rolling resistance.
    Input: Drivetrain Force (N-m)
           Speed (m/s)
    Output: Acceleration (m/s^2)
    """
    drag = Cd * rho * (speed ** 2) * Af / 2
    rollingResistance = Cr * (massCar + massDriver) * 9.81
    netThrust = accelThrust - drag - rollingResistance
    return netThrust / (massCar + massDriver)

def curRPM(speed):
    """Calculates current engine RPM.
    Based on gear ratio, tire radius, and current speed.
    Input: Speed (m/s)
    Output: Engine RPM
    """
    circum = math.pi * 2 * tireRadius
    return speed / (circum * gearRatio)

def turnSpeed(radius):
    """Calculates the turning speed of a corner based on max lateral force.
    This max lateral force was calculated based on Harrison Senor's skipad runs
    at Formula North 2018.
    TODO: Implement actual cornering models to the simulation
    Input: Turn radius (m)
    Output: Cornering Speed (m/s)
    """
    speed = math.sqrt(maxLateralLoad * radius / (massCar + massDriver))
    return speed

def straightSim(initialSpeed, length, turnEntrySpeed=100, output=False,
                braking = True):
    """Simulates a straightline run of the car.
    Input: Initial Speed (m/s)
           Length of straight (m)
           Entry Speed of the following turn (m/s)
           Output (bool) (Display graphs)
    Output: Time (secs)
    """
    # Initialize time
    curTime = 0
    timeStep = 0.01
    # Initialize variables
    rpm = 0.0
    accel = 0.0
    speed = initialSpeed
    torque = 0.0
    position = 0.0
    powerUsed = 0 # kWh
    # Empty arrays for graphing variables
    positions = []
    speeds = []
    accels = []
    time = []
    powers = []

    # Loop runs until the simulation has reached the end of the straight
    while True:
        if ((braking == True) &
            ((length - position) < brakingDistance(speed, turnEntrySpeed))):

            accel = deccelRate(speed)
        else:
            # Calculate current engine RPM
            rpm = curRPM(speed)
            # Apply engine RPM limits
            if rpm > 5000:
                rpm = 5000
            # Use engine torque map to calculate the current engine torque
            torque = peakTorq(rpm) * bamoCarLoss
            # Apply the 80 kW power limit
            power = torque * rpm / 9.5488
            if power > 80:
                power = 80
                torque = ((80 * 9.5488) / rpm) * bamoCarLoss
            # Calculate the resulting acceleration
            thrust = accelThrust(torque)
            accel = accelRate(thrust, speed)
            # Calculate power used
            powerUsed += power * timeStep

        # Append the current values to the output arrays
        accels.append(accel)
        speeds.append(speed)
        positions.append(position)
        time.append(curTime)
        powers.append(torque)
        # Based on the current acceleration, calculate the resulting speed
        # and position over the next time step
        position += (speed * timeStep) + (0.5 * accel * timeStep ** 2)
        speed += (accel * timeStep)
        # End the loop once the simulation is past the end of the straight
        if positions[-1] > length:
            break
        # Update the time stamp
        curTime += timeStep

    #print("%.2f kJ Power Used" % powerUsed)
    # Print the arrays and graph if requested by the user
    if output == True:
        #print(time)
        #print(positions)
        #print(accels)
        #print(speeds)

        fig = plt.figure()
        plt.plot(time, accels)
        plt.xlabel("Time (secs)")
        plt.ylabel("Distance (m)")
        plt.title("Acceleration Run")
        axs = fig.gca()
        #axs.axhline(0.3, color="r")
        #axs.axhline(75.3, color="r")

    # Interpolate the output arrays to find the elapsed time at the end of the
    # straight
    timeElapsed = interp.interp1d(positions, time)
    finalSpeed = interp.interp1d(positions, speeds)
    return float(timeElapsed(length))

def turnSim(radius, length):
    """Simulates the turning time of the car based on max lateral force.
    This max lateral force was calculated based on Harrison Senor's skipad runs
    at Formula North 2018.
    TODO: Implement actual cornering models to the simulation
    Input: Turn radius (m)
           Arc length (m)
    Output: Time (secs)
    """
    timeEnd = length / turnSpeed(radius)
    return timeEnd

def main():
    """Runs the main simulation loop
    When fully implemented, will load the track map and iterate through it.
    The sim will then output the resulting estimated time
    """
    """
    print("75 m acceleration time (Plus 0.3 m launch zone):")
    print("%.3f secs" % (straightSim(0, 75.3, braking=False)
                    - straightSim(0, 0.3, braking=False)))

    print("\n80 Straight into corner with 18.5 m/s entry speed:")
    print(straightSim(0, 80, turnEntrySpeed=turnSpeed(18.5),
                      output=False, braking=True))
    """
    
    print("Grand Prix Track Run")
    
    raceTime = 0
    cornerSpeeds = []
    exitSpeed = 0
    for segment in track:
        if segment[1] == 'c':
            cornerSpeeds.append(turnSpeed(segment[2]))
        else:
            cornerSpeeds.append(None)
    
    for lap in range(1,5):
        lapTime = 0
        for segment in track:
            if segment[1] == 's':
                if segment[0] < len(track) - 1:
                    elapsedTime = straightSim(exitSpeed, segment[2], turnEntrySpeed=cornerSpeeds[segment[0]+1])
                else:
                    elapsedTime = straightSim(exitSpeed, segment[2], braking=False)
                
                lapTime += elapsedTime
                #print(elapsedTime)
            else:
                exitSpeed = cornerSpeeds[segment[0]]
                elapsedTime = turnSim(segment[2], segment[3])
                lapTime += elapsedTime
                #print(elapsedTime)
        
        raceTime += lapTime
        print("Lap %d Time (secs): %.2f" % (int(lap), lapTime))
    
    print("Total Time (secs): %.2f" % raceTime)
    
    


# Set constants and run the sim
if __name__ == '__main__':
    # Purdue Grand Prix Track
    # Corners are defined: (radius, arc length)
    track = [(0,  's', 35.2806),
             (1, 'c', 8.16864, 10.54608),
             (2, 's', 22.12848),
             (3, 'c', 7.37616, 6.12648),
             (4, 's', 62.84976),
             (5, 'c', 8.90016, 28.77312),
             (6, 's', 4.1148),
             (7, 'c', 9.144, 9.84504),
             (8, 's', 8.56488),
             (9, 'c', 8.65632, 11.61288),
             (10, 's', 9.81456),
             (11, 'c', 9.54024, 16.3068),
             (12, 's', 3.90144),
             (13,  'c', 22.43328, 46.69536),
             (14,  's', 20.23872),
             (15,  'c', 11.36904, 16.58112),
             (16,  's', 8.56488),
             (17,  'c', 12.00912, 24.29256),
             (18,  's', 13.28928),
             (19,  'c', 7.95528, 12.77112),
             (20,  's', 6.5532),
             (21,  'c', 8.74776, 19.26336),
             (22,  's', 35.2806)]

    # Overall car variables and parameters
    torqRPM       = [0, 1000, 2000, 3000, 4000, 5000] # RPM
    peakTorqCurve = [240, 240, 239, 236, 235, 229] # N-m
    gearRatio = 3.2 # Unitless
    driveEfficiency = 0.85 # Percent
    bamoCarLoss = 0.85 # Percent
    tireRadius = 0.2286 # m - (16 in. wheels: 0.2032 m)
    massCar = 225 # kg
    massDriver = 80 # kg
    maxBrakingGs = 1.5
    maxLateralLoad = 7146.12 # N - Estimated from Harrison Senor's skipad runs
                             #     at Formula North 2018.
    Cd = 0.3 # Drag Coefficient
    Cr = 0.03 # Coefficient of Rolling Friction
    Af = 1.3 # m^2 - Frontal Area
    rho = 1.225 # kg/m^3 - Air Density
    # Generate a function to interpolate the torque curves
    peakTorq = interp.interp1d(torqRPM, peakTorqCurve)
    # Run the main simulation loop
    main()
