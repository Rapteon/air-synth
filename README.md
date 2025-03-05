# Air Synth

Gesture-based music synthesizer.

## Prerequisites
1. git
2. cmake (v3.25 or above)
3. *add libraries for project here*

## Build instructions
1. Clone the repository and navigate to it in a terminal.
2. Ensure you are in the `air-synth` directory and then run: `cmake -G "Unix Makefiles" -B build`  
This will use the **Unix Makefiles** generator to create makefiles for compiling the project  
in the **build** folder. If you want to use the default generator, run the command: `cmake -B build`
3. To build the project, you can use this command:  
    ```
    cmake --build ./build --config Release --target AirSynth
    ```
