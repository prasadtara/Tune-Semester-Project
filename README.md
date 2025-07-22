This Python project simulates engine performance data, with a focus on accurately calculating horsepower-related values and verifying those through unit tests. 
It is intended to help test and visualize various engine characteristics such as MAP (Manifold Absolute Pressure), RPM, and horsepower over a simulated period.'

Features:
- Simulates engine data over a user-specified duration.
- Calculates and stores a Horsepower Unit Factor for internal performance comparisons.
- Includes unit tests using Python's unittest framework.
- Logs and prints output for transparency and debugging

You can use an IDE like PyCharm, VS Code, Jupyter Notebook to run this program.
- No external dependencies are required for basic simulation and testing

Regarding User inputs:
- When you run the program, it will ask for the following inputs, one at a time:
    Elevation (meters)
        - Enter the altitude in meters where the engine operates (e.g., 0 for sea level, 1500 for high altitude).
        - Range accepted: -400 to 8000 meters
    Naturally aspirated peak horsepower
        - Enter the peak horsepower your naturally aspirated engine produces (e.g., 300).
        - Must be at least 1.
    Target peak boost pressure (PSI)
         - Enter the maximum boost pressure your engine’s boost system can produce in PSI, including atmospheric pressure
           (e.g., if atmospheric pressure is 14.7 PSI and your turbo adds 6 PSI, enter 20.7).
  -         [NOTE: You must do research on the modification you're planning to
               make and enter those parameters accordingly. This means that you
               must do your research and calculate with the PSI of boost that the
               turbo that YOU want to add to your vehicle would make. This can be
               done with a simple Google Search '(make and model of your preferred
               boost) boost pressure in PSI.' This is also catered to YOU, so you,
               as the client must provide the parameters for the modificaiton or
               upgrade that you are interested in so we can help you get the right
               data to decide whether or not it's an upgrade that you would be
               satisfied with if you invested in it legitimately.]
       -  Must be at least the atmospheric pressure, max 45 PSI.
    Redline RPM
        - Enter the maximum RPM your engine reaches (e.g., 7000).
        - Range accepted: 5000 to 10000 RPM
    Idle RPM
        - Enter the RPM your engine idles at (e.g., 800).
        - Range accepted: 500 to 1000 RPM
 
What Happens After All the Input is Provided?
- The program will calculate internal parameters based on your inputs.
- It will run a 45-second simulation in the background showing different engine states
  (idle, cruise, acceleration, deceleration) with dynamic RPM, MAP, and throttle position values.
- A live graph of manifold absolute pressure over time is updated during simulation.
- When the simulation finishes, a graph image named engine_simulation.png will be saved in the current directory.
- The program prints confirmation that the simulation is complete and where the image is saved.


