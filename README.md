# Air Synth

Gesture-based music synthesizer.
Checkout the [Website](https://rapteon.github.io/air-synth/) for more details!!

## Build instructions
1. Clone the repository and navigate to it in a terminal.
2. Ensure you are in the `air-synth` directory and then run: `cmake -G "Unix Makefiles" -B build`  
This will use the **Unix Makefiles** generator to create makefiles for compiling the project  
in the **build** folder. If you want to use the default generator, run the command: `cmake -B build`
3. To build the project, you can use this command:  
    ```
    cmake --build ./build --config Release --target AirSynth
    ```
## Running the code
The sensors and the buttons must be connected to the Raspberry Pi before running the program.  
Please see the [hardware section on the Wiki home page](https://github.com/Rapteon/air-synth/wiki#hardware)
for the circuit diagram.

## Additional information

To know more about the idea, see [About](https://github.com/Rapteon/air-synth/wiki/About).  
[Link to Wiki](https://github.com/Rapteon/air-synth/wiki)
