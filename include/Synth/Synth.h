#ifndef MPU_SYNTH_H
#define MPU_SYNTH_H

#include "IMU/MPU6050.h"
#include "Button/Button.h"
#include <queue>
#include <vector>
#include <array>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <cmath>
#include <iostream>
#include <map>

// STK includes
#include <stk/Instrmnt.h>
#include <stk/Rhodey.h>
#include <stk/Wurley.h>
#include <stk/Moog.h>
#include <stk/Clarinet.h>
#include <stk/RtAudio.h>
#include <stk/Stk.h>

// Define constants
#define ACCEL_SCALE 16384.0
#define G 9.81
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#include <queue>

// Scale type definitions
enum class ScaleType {
    MAJOR,
    MINOR,
    PENTATONIC,
    BLUES,
    CHROMATIC
};

// Instrument type definitions
enum class InstrumentType {
    PIANO,
    RHODEY,
    WURLEY,
    MOOG,
    CLARINET
};

class MPUSynth: public MPU6050::ControllerCallbackInterface {
public:
    MPUSynth(int sampleRate = 44100);
    ~MPUSynth();

    // Start and stop audio processing
    void start();
    void stop();

    // Set parameters
    void setInstrument(InstrumentType instrumentType);
    void setScale(ScaleType scaleType, const std::string& rootNote = "C");
    void setVolume(float volume);
    void setNoteDuration(float milliseconds);
    
    // Mappings
    void setMappingMode(bool xAxis, bool yAxis, bool zAxis);
    void setAxisNoteSpreads(int xSpread, int ySpread, int zSpread);
    void setAxisVelocitySensitivity(float xSens, float ySens, float zSens);

    virtual void hasEvent(ControllerEvent &e) {
        // TODO do something here based on x, y and z values sent by controller.
        // Append the event to the queue.
    }
private:
    // Note generation
    void generateNotes(const std::string& rootNote, ScaleType scaleType);
    int noteToMidiNumber(const std::string& note);
    float midiNumberToFrequency(int midiNumber);
    
    // Audio callback and processing
    static int audioCallback(void* outputBuffer, void* inputBuffer,
                           unsigned int nBufferFrames,
                           double streamTime, RtAudioStreamStatus status,
                           void* userData);
    void processSample(float* buffer, unsigned int nBufferFrames);
    
    // MPU6050 sensor data processing
    void processMpuData();
    void mapAccelerationToNotes(double x, double y, double z);
    bool shouldTriggerNote(double acceleration, double prevAcceleration, double threshold);
    
    // STK and RtAudio related members
    stk::Instrmnt* instrument;
    RtAudio dac;
    
    // Audio parameters
    int sampleRate;
    float volume;
    float noteDuration; // in milliseconds
    std::atomic<bool> noteActive;
    std::atomic<float> currentFrequency;
    std::atomic<float> noteAmplitude;
    std::atomic<float> noteTime; // current time within note duration
    
    // Note mapping parameters
    std::vector<float> noteFrequencies;
    std::array<bool, 3> axisEnabled; // x, y, z
    std::array<int, 3> axisNoteSpread; // number of notes in range for each axis
    std::array<float, 3> axisVelocitySensitivity;
    
    // Sensor related
    MPU6050* mpuSensor;
    std::array<double, 3> prevAcceleration;
    std::array<double, 3> accelerationThresholds;
    
    // Thread control
    std::atomic<bool> running;
    std::thread sensorThread;
    std::mutex instrumentMutex;
    std::queue<ControllerEvent> eventQueue;
};

#endif // MPU_SYNTH_H