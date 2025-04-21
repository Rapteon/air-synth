#ifndef SYNTH_H
#define SYNTH_H

#include <vector>
#include <map>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <cmath>
#include <memory>
#include <stk/RtAudio.h>
#include <stk/Rhodey.h>
#include <stk/Wurley.h>
#include <stk/Moog.h>
#include <stk/Clarinet.h>
#include <stk/Stk.h>
#include "IMU/MPU6050.h"

// Forward declarations
class MPU6050;
class ControllerEvent;

/**
 * @brief Enumeration of available musical scales.
 */
enum class ScaleType {
    MAJOR,      ///< Major scale (Ionian)
    MINOR,      ///< Natural minor scale (Aeolian)
    PENTATONIC, ///< Pentatonic scale
    BLUES,      ///< Blues scale
    CHROMATIC   ///< Chromatic scale
};

/**
 * @brief Enumeration of available instruments.
 */
enum class InstrumentType {
    PIANO,      ///< Piano (Rhodey)
    RHODEY,     ///< Rhodes electric piano
    WURLEY,     ///< Wurlitzer electric piano
    MOOG,       ///< Moog synthesizer
    CLARINET    ///< Clarinet
};

// Expanded list of available root notes
const std::vector<std::string> AVAILABLE_ROOT_NOTES = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

// Expanded list of available scale types
const std::map<std::string, ScaleType> AVAILABLE_SCALES = {
    {"Major", ScaleType::MAJOR},
    {"Minor", ScaleType::MINOR},
    {"Pentatonic", ScaleType::PENTATONIC},
    {"Blues", ScaleType::BLUES},
    {"Chromatic", ScaleType::CHROMATIC}
};

// Expanded list of available instruments
const std::map<std::string, InstrumentType> AVAILABLE_INSTRUMENTS = {
    {"Piano", InstrumentType::PIANO},
    {"Rhodes", InstrumentType::RHODEY},
    {"Wurlitzer", InstrumentType::WURLEY},
    {"Moog", InstrumentType::MOOG},
    {"Clarinet", InstrumentType::CLARINET}
};

/**
 * @class MPUSynth
 * @brief Synthesizer class that generates sound based on MPU6050 motion data.
 * 
 * This class handles audio synthesis using the STK library, mapping motion data
 * to musical notes and controlling the sound generation. It implements proper
 * resource management and thread synchronization.
 */
class MPUSynth: public MPU6050::ControllerCallbackInterface {
private:
    MPU6050* mpuSensor;         ///< Reference to the MPU6050 sensor
    const int sampleRate;        ///< Audio sample rate
    std::atomic<float> volume;   ///< Overall volume (0.0 to 1.0)
    std::atomic<float> noteDuration;  ///< Duration of each note (ms)
    
    std::atomic<bool> noteActive;      ///< Is a note currently playing?
    std::atomic<float> currentFrequency;  ///< Current note frequency
    std::atomic<float> noteAmplitude;     ///< Current note amplitude
    std::atomic<float> noteTime;         ///< Current note time position (ms)
    
    std::vector<float> noteFrequencies;  ///< Available note frequencies
    
    // Axis mapping settings
    std::vector<bool> axisEnabled;      ///< Which axes trigger notes
    std::vector<int> axisNoteSpread;    ///< How many notes to map to each axis
    std::vector<float> axisVelocitySensitivity;  ///< Sensitivity per axis
    
    // Acceleration thresholds
    std::vector<double> accelerationThresholds;
    
    // Previous acceleration values
    std::vector<double> prevAcceleration;
    
    // Continuous mode flag
    std::atomic<bool> continuousMode;
    
    // Audio output
    RtAudio dac;
    
    // STK instrument
    std::unique_ptr<stk::Instrmnt> instrument;
    std::mutex instrumentMutex;
    
    // Sensor processing thread
    std::thread sensorThread;
    std::atomic<bool> running;
    
    // Event queue
    std::queue<ControllerEvent> eventQueue;
    std::mutex queueMutex;
    
    // Generate notes based on scale
    void generateNotes(const std::string& rootNote, ScaleType scaleType);
    
    // Helper methods for note generation
    int noteToMidiNumber(const std::string& note) const;
    float midiNumberToFrequency(int midiNumber) const;
    
    // Audio callback
    static int audioCallback(void* outputBuffer, void* inputBuffer,
                           unsigned int nBufferFrames,
                           double streamTime, RtAudioStreamStatus status,
                           void* userData);
    
    // Process audio samples
    void processSample(float* buffer, unsigned int nBufferFrames);
    
    // Process sensor data
    void processMpuData();
    
    // Map acceleration to notes
    void mapAccelerationToNotes(double x, double y, double z);
    
    // Determine if a note should be triggered
    bool shouldTriggerNote(double acceleration, double prevAcceleration, double threshold) const;

public:
    /**
     * @brief Constructs a new MPUSynth instance.
     * @param mpuSensor Pointer to the MPU6050 sensor
     * @param sampleRate Audio sample rate in Hz
     */
    MPUSynth(MPU6050* mpuSensor, int sampleRate);
    
    /**
     * @brief Destructor ensures proper cleanup of resources.
     */
    ~MPUSynth();
    
    // Delete copy constructor and assignment operator
    MPUSynth(const MPUSynth&) = delete;
    MPUSynth& operator=(const MPUSynth&) = delete;
    
    virtual void hasEvent(ControllerEvent &e) override {
        // Add the event to the processing queue
        addEvent(e);
    }

    /**
     * @brief Starts the synthesizer.
     * @throws std::runtime_error if audio initialization fails
     */
    void start();
    
    /**
     * @brief Stops the synthesizer.
     */
    void stop();
    
    /**
     * @brief Sets the instrument type.
     * @param instrumentType Type of instrument to use
     */
    void setInstrument(InstrumentType instrumentType);
    
    /**
     * @brief Sets the musical scale.
     * @param scaleType Type of scale to use
     * @param rootNote Root note of the scale
     */
    void setScale(ScaleType scaleType, const std::string& rootNote);
    
    /**
     * @brief Sets the overall volume.
     * @param vol Volume level (0.0 to 1.0)
     */
    void setVolume(float vol);
    
    /**
     * @brief Sets the note duration.
     * @param milliseconds Duration in milliseconds
     */
    void setNoteDuration(float milliseconds);
    
    /**
     * @brief Sets which axes trigger notes.
     * @param xAxis Enable X-axis
     * @param yAxis Enable Y-axis
     * @param zAxis Enable Z-axis
     */
    void setMappingMode(bool xAxis, bool yAxis, bool zAxis);
    
    /**
     * @brief Sets how many notes to map to each axis.
     * @param xSpread Number of notes for X-axis
     * @param ySpread Number of notes for Y-axis
     * @param zSpread Number of notes for Z-axis
     */
    void setAxisNoteSpreads(int xSpread, int ySpread, int zSpread);
    
    /**
     * @brief Sets the velocity sensitivity for each axis.
     * @param xSens X-axis sensitivity
     * @param ySens Y-axis sensitivity
     * @param zSens Z-axis sensitivity
     */
    void setAxisVelocitySensitivity(float xSens, float ySens, float zSens);
    
    /**
     * @brief Sets continuous mode.
     * @param continuous True to enable continuous mode
     */
    void setContinuousMode(bool continuous) { continuousMode = continuous; }
    
    /**
     * @brief Gets the continuous mode state.
     * @return True if continuous mode is enabled
     */
    bool isContinuousMode() const { return continuousMode.load(); }
    
    /**
     * @brief Adds an event to the processing queue.
     * @param event The event to add
     */
    void addEvent(const ControllerEvent& event);
};

#endif // SYNTH_H