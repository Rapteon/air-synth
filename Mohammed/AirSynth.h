/**
 * @file AirSynth.h
 * @brief Interface for the Air Synth gesture-controlled synthesizer
 * @author Mohammed
 * @date Feb 25, 2025
 * 
 * This file provides an interface for a gesture-controlled synthesizer that
 * uses the MPU6050 sensor to detect hand movements and generate sounds.
 */

#ifndef AIR_SYNTH_H
#define AIR_SYNTH_H

#include "MPU6050.h"

// Pin definitions
#define SPEAKER_PIN 9  // PWM pin for speaker output

// Note frequency definitions (Hz) - A4 major scale
#define NOTE_A4  440.00
#define NOTE_B4  493.88
#define NOTE_CS5 554.37
#define NOTE_D5  587.33
#define NOTE_E5  659.25
#define NOTE_FS5 739.99
#define NOTE_GS5 830.61
#define NOTE_A5  880.00

// Gesture thresholds for triggering notes
#define GESTURE_THRESHOLD_LOW  0.5f   // g units
#define GESTURE_THRESHOLD_MED  1.0f   // g units
#define GESTURE_THRESHOLD_HIGH 1.5f   // g units

// Gesture debounce time (ms)
#define GESTURE_DEBOUNCE_TIME 200

/**
 * @brief Enumeration of detected gesture types
 */
enum GestureType {
    GESTURE_NONE,
    GESTURE_TILT_LEFT,
    GESTURE_TILT_RIGHT,
    GESTURE_TILT_FORWARD,
    GESTURE_TILT_BACKWARD,
    GESTURE_SHAKE_X,
    GESTURE_SHAKE_Y,
    GESTURE_SHAKE_Z,
    GESTURE_PUNCH
};

/**
 * @brief Enumeration of synthesizer modes
 */
enum SynthMode {
    MODE_DISCRETE,   // Play discrete notes
    MODE_CONTINUOUS, // Continuous pitch variation
    MODE_DRUM,       // Percussion sounds
    MODE_CHORD       // Play chords
};

/**
 * @class AirSynth
 * @brief Class for the Air Synth gesture-controlled synthesizer
 */
class AirSynth {
public:
    /**
     * @brief Constructor
     * @param mpu Reference to an MPU6050 instance
     */
    AirSynth(MPU6050 &mpu);
    
    /**
     * @brief Initialize the Air Synth
     * @return True if initialization successful, false otherwise
     */
    bool begin();
    
    /**
     * @brief Process sensor data and generate sound
     * Call this method in the main loop
     */
    void update();
    
    /**
     * @brief Set the synthesizer mode
     * @param mode The desired synth mode
     */
    void setMode(SynthMode mode);
    
    /**
     * @brief Get the current synthesizer mode
     * @return The current synth mode
     */
    SynthMode getMode() const;
    
    /**
     * @brief Set the base volume (amplitude)
     * @param volume Volume level from 0.0 to 1.0
     */
    void setVolume(float volume);
    
    /**
     * @brief Get the current volume
     * @return Current volume level from 0.0 to 1.0
     */
    float getVolume() const;
    
    /**
     * @brief Enable or disable sound output
     * @param enable True to enable, false to disable
     */
    void enableSound(bool enable);
    
    /**
     * @brief Check if sound is enabled
     * @return True if sound is enabled, false otherwise
     */
    bool isSoundEnabled() const;
    
    /**
     * @brief Get the last detected gesture
     * @return The last detected gesture type
     */
    GestureType getLastGesture() const;
    
    /**
     * @brief Get the current playing frequency
     * @return The current playing frequency in Hz, or 0 if no sound is playing
     */
    float getCurrentFrequency() const;

private:
    MPU6050 &_mpu;                // Reference to the MPU6050 sensor
    SynthMode _mode;              // Current synth mode
    float _volume;                // Current volume level (0.0 to 1.0)
    bool _soundEnabled;           // Whether sound output is enabled
    GestureType _lastGesture;     // Last detected gesture
    float _currentFrequency;      // Current playing frequency in Hz
    unsigned long _lastGestureTime; // Timestamp of the last gesture detection
    
    // Smoothing filter variables for gesture detection
    float _filteredAx, _filteredAy, _filteredAz;
    float _filteredGx, _filteredGy, _filteredGz;
    
    /**
     * @brief Detect gestures from sensor data
     * @param data Sensor data from the MPU6050
     * @return Detected gesture type
     */
    GestureType detectGesture(const MPU6050::SensorData &data);
    
    /**
     * @brief Play a tone based on the detected gesture
     * @param gesture The detected gesture
     */
    void playGestureSound(GestureType gesture);
    
    /**
     * @brief Generate a tone with the specified frequency
     * @param frequency Frequency in Hz
     */
    void playTone(float frequency);
    
    /**
     * @brief Stop playing any current tone
     */
    void stopTone();
    
    /**
     * @brief Apply low-pass filter to sensor readings
     * @param data Raw sensor data
     * @return Filtered sensor data
     */
    MPU6050::SensorData filterSensorData(const MPU6050::SensorData &data);
    
    /**
     * @brief Map an accelerometer or gyroscope value to a frequency
     * @param value The sensor value to map
     * @param minValue Minimum expected sensor value
     * @param maxValue Maximum expected sensor value
     * @param minFreq Minimum frequency in the output range
     * @param maxFreq Maximum frequency in the output range
     * @return Mapped frequency value
     */
    float mapToFrequency(float value, float minValue, float maxValue,
                         float minFreq, float maxFreq);
};

#endif // AIR_SYNTH_H