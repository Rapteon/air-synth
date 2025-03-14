/**
 * @file AirSynth.cpp
 * @brief Implementation of the Air Synth gesture-controlled synthesizer
 * @author Mohammed
 * @date Feb 25, 2025
 */

#include "AirSynth.h"

// Filter coefficient for smoothing sensor readings (0.0 to 1.0)
// Lower values provide more smoothing, higher values more responsiveness
#define FILTER_ALPHA 0.2f

// Frequency ranges for continuous pitch mapping
#define MIN_FREQ 220.0f  // A3
#define MAX_FREQ 880.0f  // A5

AirSynth::AirSynth(MPU6050 &mpu) : 
    _mpu(mpu),
    _mode(MODE_DISCRETE),
    _volume(0.5f),
    _soundEnabled(true),
    _lastGesture(GESTURE_NONE),
    _currentFrequency(0.0f),
    _lastGestureTime(0),
    _filteredAx(0.0f),
    _filteredAy(0.0f),
    _filteredAz(0.0f),
    _filteredGx(0.0f),
    _filteredGy(0.0f),
    _filteredGz(0.0f) {
}

bool AirSynth::begin() {
    // Set up the speaker pin as output
    pinMode(SPEAKER_PIN, OUTPUT);
    
    // Initialize with no sound
    stopTone();
    
    return true;
}

void AirSynth::update() {
    // Read sensor data
    MPU6050::SensorData data = _mpu.readSensorData();
    
    // Apply filtering to smooth the readings
    data = filterSensorData(data);
    
    // Detect gestures
    GestureType gesture = detectGesture(data);
    
    // Process the gesture based on the current mode
    if (gesture != GESTURE_NONE) {
        // Store the detected gesture
        _lastGesture = gesture;
        _lastGestureTime = millis();
        
        // Generate sound if enabled
        if (_soundEnabled) {
            playGestureSound(gesture);
        }
    } else if (_mode == MODE_CONTINUOUS) {
        // In continuous mode, update the pitch even without distinct gestures
        // Map pitch based on tilt angle (using X-axis acceleration)
        if (_soundEnabled && abs(data.ax) > GESTURE_THRESHOLD_LOW) {
            float mappedFreq = mapToFrequency(data.ax, 
                                             -GESTURE_THRESHOLD_HIGH, 
                                              GESTURE_THRESHOLD_HIGH,
                                              MIN_FREQ, 
                                              MAX_FREQ);
            playTone(mappedFreq);
        } else {
            stopTone();
        }
    }
}

void AirSynth::setMode(SynthMode mode) {
    _mode = mode;
    
    // Stop any playing tone when changing modes
    stopTone();
}

SynthMode AirSynth::getMode() const {
    return _mode;
}

void AirSynth::setVolume(float volume) {
    // Clamp volume to valid range
    _volume = constrain(volume, 0.0f, 1.0f);
}

float AirSynth::getVolume() const {
    return _volume;
}

void AirSynth::enableSound(bool enable) {
    _soundEnabled = enable;
    
    // Stop any playing tone if disabling sound
    if (!enable) {
        stopTone();
    }
}

bool AirSynth::isSoundEnabled() const {
    return _soundEnabled;
}

GestureType AirSynth::getLastGesture() const {
    return _lastGesture;
}

float AirSynth::getCurrentFrequency() const {
    return _currentFrequency;
}

GestureType AirSynth::detectGesture(const MPU6050::SensorData &data) {
    // Check if enough time has passed since the last gesture detection
    if (millis() - _lastGestureTime < GESTURE_DEBOUNCE_TIME) {
        return GESTURE_NONE;
    }
    
    // Detect punch (rapid acceleration along Z-axis)
    if (data.az > GESTURE_THRESHOLD_HIGH) {
        return GESTURE_PUNCH;
    }
    
    // Detect shaking motions (using gyroscope data)
    if (abs(data.gx) > 100.0f) {
        return GESTURE_SHAKE_X;
    }
    if (abs(data.gy) > 100.0f) {
        return GESTURE_SHAKE_Y;
    }
    if (abs(data.gz) > 100.0f) {
        return GESTURE_SHAKE_Z;
    }
    
    // Detect tilting (using accelerometer data)
    if (data.ax < -GESTURE_THRESHOLD_MED) {
        return GESTURE_TILT_RIGHT;
    }
    if (data.ax > GESTURE_THRESHOLD_MED) {
        return GESTURE_TILT_LEFT;
    }
    if (data.ay < -GESTURE_THRESHOLD_MED) {
        return GESTURE_TILT_BACKWARD;
    }
    if (data.ay > GESTURE_THRESHOLD_MED) {
        return GESTURE_TILT_FORWARD;
    }
    
    // No gesture detected
    return GESTURE_NONE;
}

void AirSynth::playGestureSound(GestureType gesture) {
    switch (_mode) {
        case MODE_DISCRETE:
            // Play discrete notes based on gestures
            switch (gesture) {
                case GESTURE_TILT_LEFT:
                    playTone(NOTE_A4);
                    break;
                case GESTURE_TILT_RIGHT:
                    playTone(NOTE_B4);
                    break;
                case GESTURE_TILT_FORWARD:
                    playTone(NOTE_CS5);
                    break;
                case GESTURE_TILT_BACKWARD:
                    playTone(NOTE_D5);
                    break;
                case GESTURE_SHAKE_X:
                    playTone(NOTE_E5);
                    break;
                case GESTURE_SHAKE_Y:
                    playTone(NOTE_FS5);
                    break;
                case GESTURE_SHAKE_Z:
                    playTone(NOTE_GS5);
                    break;
                case GESTURE_PUNCH:
                    playTone(NOTE_A5);
                    break;
                default:
                    stopTone();
                    break;
            }
            break;
            
        case MODE_DRUM:
            // Play percussion sounds based on gestures
            // For simplicity, we'll use short beeps of different frequencies
            // In a real project, you might use sampled drum sounds
            switch (gesture) {
                case GESTURE_TILT_LEFT:
                    playTone(150.0f); // Low drum
                    delay(50);
                    stopTone();
                    break;
                case GESTURE_TILT_RIGHT:
                    playTone(200.0f); // Mid drum
                    delay(50);
                    stopTone();
                    break;
                case GESTURE_SHAKE_X:
                case GESTURE_SHAKE_Y:
                case GESTURE_SHAKE_Z:
                    playTone(800.0f); // Hi-hat
                    delay(30);
                    stopTone();
                    break;
                case GESTURE_PUNCH:
                    playTone(100.0f); // Bass drum
                    delay(100);
                    stopTone();
                    break;
                default:
                    break;
            }
            break;
            
        case MODE_CHORD:
            // Play chords based on gestures
            // For simplicity, we'll use a single note to represent a chord
            // In a real project, you might use multiple tones simultaneously
            switch (gesture) {
                case GESTURE_TILT_LEFT:
                    playTone(NOTE_A4); // A major chord
                    break;
                case GESTURE_TILT_RIGHT:
                    playTone(NOTE_E5); // E major chord
                    break;
                case GESTURE_TILT_FORWARD:
                    playTone(NOTE_D5); // D major chord
                    break;
                case GESTURE_TILT_BACKWARD:
                    playTone(NOTE_A5); // A major chord (higher octave)
                    break;
                default:
                    stopTone();
                    break;
            }
            break;
            
        case MODE_CONTINUOUS:
            // Handled in the update method for continuous pitch control
            break;
    }
}

void AirSynth::playTone(float frequency) {
    if (frequency <= 0.0f) {
        stopTone();
        return;
    }
    
    _currentFrequency = frequency;
    
    // On Arduino, we can use the tone() function to generate a square wave
    // The volume control would ideally use PWM, but for simplicity,
    // we're just using the tone function here
    tone(SPEAKER_PIN, (unsigned int)frequency);
}

void AirSynth::stopTone() {
    _currentFrequency = 0.0f;
    noTone(SPEAKER_PIN);
}

MPU6050::SensorData AirSynth::filterSensorData(const MPU6050::SensorData &data) {
    // Simple low-pass filter to smooth the readings
    _filteredAx = _filteredAx * (1.0f - FILTER_ALPHA) + data.ax * FILTER_ALPHA;
    _filteredAy = _filteredAy * (1.0f - FILTER_ALPHA) + data.ay * FILTER_ALPHA;
    _filteredAz = _filteredAz * (1.0f - FILTER_ALPHA) + data.az * FILTER_ALPHA;
    _filteredGx = _filteredGx * (1.0f - FILTER_ALPHA) + data.gx * FILTER_ALPHA;
    _filteredGy = _filteredGy * (1.0f - FILTER_ALPHA) + data.gy * FILTER_ALPHA;
    _filteredGz = _filteredGz * (1.0f - FILTER_ALPHA) + data.gz * FILTER_ALPHA;
    
    // Create a new data structure with the filtered values
    MPU6050::SensorData filtered;
    filtered.ax = _filteredAx;
    filtered.ay = _filteredAy;
    filtered.az = _filteredAz;
    filtered.gx = _filteredGx;
    filtered.gy = _filteredGy;
    filtered.gz = _filteredGz;
    filtered.temp = data.temp; // No need to filter temperature
    
    return filtered;
}

float AirSynth::mapToFrequency(float value, float minValue, float maxValue,
                               float minFreq, float maxFreq) {
    // Constrain the input value to the expected range
    float constrainedValue = constrain(value, minValue, maxValue);
    
    // Map the input range to the frequency range
    return minFreq + (constrainedValue - minValue) * (maxFreq - minFreq) / (maxValue - minValue);
}