#include "Synth/Synth.h"

// Root note frequencies (A4 = 440 Hz)
const std::map<std::string, int> NOTE_TO_MIDI = {
    {"C", 60}, {"C#", 61}, {"Db", 61}, {"D", 62}, 
    {"D#", 63}, {"Eb", 63}, {"E", 64}, {"F", 65}, 
    {"F#", 66}, {"Gb", 66}, {"G", 67}, {"G#", 68}, 
    {"Ab", 68}, {"A", 69}, {"A#", 70}, {"Bb", 70}, {"B", 71}
};

// Scale intervals (half steps)
const std::map<ScaleType, std::vector<int>> SCALE_INTERVALS = {
    {ScaleType::MAJOR, {0, 2, 4, 5, 7, 9, 11, 12}},
    {ScaleType::MINOR, {0, 2, 3, 5, 7, 8, 10, 12}},
    {ScaleType::PENTATONIC, {0, 2, 4, 7, 9, 12, 14, 16}},
    {ScaleType::BLUES, {0, 3, 5, 6, 7, 10, 12, 15}},
    {ScaleType::CHROMATIC, {0, 1, 2, 3, 4, 5, 6, 7}}
};

MPUSynth::MPUSynth(MPU6050* mpuSensor, int sampleRate)
    : mpuSensor(mpuSensor), 
      sampleRate(sampleRate),
      volume(0.5f),
      noteDuration(250.0f), // 250ms default note duration
      noteActive(false),
      currentFrequency(440.0f),
      noteAmplitude(0.0f),
      noteTime(0.0f),
      running(false),
      instrument(nullptr)
{
    // Initialize STK - use the static method instead of instantiating
    stk::Stk::setSampleRate(sampleRate);
    
    // Default instrument is a piano (Rhodey)
    instrument = new stk::Rhodey();
    
    // Default scale is C major
    generateNotes("C", ScaleType::MAJOR);
    
    // Default mapping settings
    axisEnabled = {true, true, true}; // Enable all axes
    axisNoteSpread = {8, 8, 8}; // Each axis can trigger all 8 notes
    axisVelocitySensitivity = {1.0f, 1.0f, 1.0f}; // Default sensitivity
    
    // Default acceleration thresholds
    accelerationThresholds = {3.0, 3.0, 2.0};
    
    // Initialize previous acceleration values
    prevAcceleration = {0.0, 0.0, 0.0};
}

MPUSynth::~MPUSynth() {
    stop();
    
    if (instrument) {
        delete instrument;
        instrument = nullptr;
    }
}

void MPUSynth::start() {
    if (running) return;
    
    // Initialize RtAudio
    RtAudio::StreamParameters parameters;
    parameters.deviceId = dac.getDefaultOutputDevice();
    parameters.nChannels = 1;
    unsigned int bufferFrames = 256;  // 256 sample frames
    
    try {
        dac.openStream(&parameters, nullptr, RTAUDIO_FLOAT32,
                       sampleRate, &bufferFrames, &MPUSynth::audioCallback, this);
        dac.startStream();
    }
    catch (RtAudioError& e) {
        std::cerr << "RtAudio error: " << e.getMessage() << std::endl;
        return;
    }
    
    // Start sensor processing thread
    running = true;
    sensorThread = std::thread(&MPUSynth::processMpuData, this);
    
    std::cout << "MPUSynth started successfully" << std::endl;
}

void MPUSynth::stop() {
    // Stop sensor thread
    if (running) {
        running = false;
        if (sensorThread.joinable()) {
            sensorThread.join();
        }
    }
    
    // Stop audio stream
    if (dac.isStreamOpen()) {
        try {
            dac.stopStream();
        }
        catch (RtAudioError& e) {
            std::cerr << "RtAudio error: " << e.getMessage() << std::endl;
        }
        
        if (dac.isStreamOpen()) {
            dac.closeStream();
        }
    }
    
    std::cout << "MPUSynth stopped" << std::endl;
}

void MPUSynth::setInstrument(InstrumentType instrumentType) {
    std::lock_guard<std::mutex> lock(instrumentMutex);
    
    if (instrument) {
        delete instrument;
        instrument = nullptr;
    }
    
    switch (instrumentType) {
        case InstrumentType::PIANO:
        case InstrumentType::RHODEY:
            instrument = new stk::Rhodey();
            break;
        case InstrumentType::WURLEY:
            instrument = new stk::Wurley();
            break;
        case InstrumentType::MOOG:
            instrument = new stk::Moog();
            break;
        case InstrumentType::CLARINET:
            instrument = new stk::Clarinet();
            break;
        default:
            instrument = new stk::Rhodey();
            break;
    }
}

void MPUSynth::setScale(ScaleType scaleType, const std::string& rootNote) {
    generateNotes(rootNote, scaleType);
}

void MPUSynth::setVolume(float vol) {
    volume = std::max(0.0f, std::min(1.0f, vol));
}

void MPUSynth::setNoteDuration(float milliseconds) {
    noteDuration = std::max(50.0f, std::min(2000.0f, milliseconds));
}

void MPUSynth::setMappingMode(bool xAxis, bool yAxis, bool zAxis) {
    axisEnabled = {xAxis, yAxis, zAxis};
}

void MPUSynth::setAxisNoteSpreads(int xSpread, int ySpread, int zSpread) {
    axisNoteSpread = {
        std::max(1, std::min(8, xSpread)),
        std::max(1, std::min(8, ySpread)),
        std::max(1, std::min(8, zSpread))
    };
}

void MPUSynth::setAxisVelocitySensitivity(float xSens, float ySens, float zSens) {
    axisVelocitySensitivity = {
        std::max(0.1f, std::min(5.0f, xSens)),
        std::max(0.1f, std::min(5.0f, ySens)),
        std::max(0.1f, std::min(5.0f, zSens))
    };
}

void MPUSynth::generateNotes(const std::string& rootNote, ScaleType scaleType) {
    noteFrequencies.clear();
    
    // Get MIDI number for root note
    int rootMidi = noteToMidiNumber(rootNote);
    if (rootMidi < 0) {
        rootMidi = 60; // Default to middle C if note not found
    }
    
    // Get scale intervals
    auto intervals = SCALE_INTERVALS.find(scaleType);
    if (intervals == SCALE_INTERVALS.end()) {
        // Default to major scale if scale type not found
        intervals = SCALE_INTERVALS.find(ScaleType::MAJOR);
    }
    
    // Generate frequencies for each note in the scale
    for (int interval : intervals->second) {
        int midiNumber = rootMidi + interval;
        float frequency = midiNumberToFrequency(midiNumber);
        noteFrequencies.push_back(frequency);
    }
}

int MPUSynth::noteToMidiNumber(const std::string& note) {
    auto it = NOTE_TO_MIDI.find(note);
    if (it != NOTE_TO_MIDI.end()) {
        return it->second;
    }
    return -1; // Note not found
}

float MPUSynth::midiNumberToFrequency(int midiNumber) {
    // A4 (MIDI 69) = 440Hz
    return 440.0f * std::pow(2.0f, (midiNumber - 69) / 12.0f);
}

int MPUSynth::audioCallback(void* outputBuffer, void* inputBuffer,
                          unsigned int nBufferFrames,
                          double streamTime, RtAudioStreamStatus status,
                          void* userData) {
    MPUSynth* synth = static_cast<MPUSynth*>(userData);
    float* buffer = static_cast<float*>(outputBuffer);
    
    if (status == RTAUDIO_OUTPUT_UNDERFLOW) {
        std::cerr << "RtAudio underflow detected!" << std::endl;
    }
    
    synth->processSample(buffer, nBufferFrames);
    return 0;
}

void MPUSynth::processSample(float* buffer, unsigned int nBufferFrames) {
    std::lock_guard<std::mutex> lock(instrumentMutex);
    
    for (unsigned int i = 0; i < nBufferFrames; i++) {
        if (noteActive) {
            // Update note envelope
            if (noteTime < 10.0f) {
                // Attack (quick ramp up)
                noteAmplitude = (noteTime / 10.0f) * volume;
            }
            else if (noteTime > (noteDuration - 50.0f)) {
                // Release (fade out)
                float releasePhase = (noteDuration - noteTime) / 50.0f;
                noteAmplitude = std::max(0.0f, releasePhase * volume);
            }
            
            // Increment note time - use atomic store/load since += isn't defined for std::atomic
            float currentTime = noteTime.load();
            noteTime.store(currentTime + (1000.0f / sampleRate));  // Convert to ms
            
            // Check if note is finished
            if (noteTime >= noteDuration) {
                noteActive = false;
                noteAmplitude = 0.0f;
            }
        }
        
        // Generate and process audio sample
        if (instrument) {
            instrument->setFrequency(currentFrequency);
            buffer[i] = instrument->tick() * noteAmplitude;
        } else {
            buffer[i] = 0.0f;
        }
    }
}

void MPUSynth::processMpuData() {
    while (running) {
        // Process only if sensor is active
        if (mpuSensor != nullptr) {
            // Read accelerometer values
            double accelX = (mpuSensor->read_word(ACCEL_XOUT_H) / ACCEL_SCALE) * G;
            double accelY = (mpuSensor->read_word(ACCEL_YOUT_H) / ACCEL_SCALE) * G;
            double accelZ = (mpuSensor->read_word(ACCEL_ZOUT_H) / ACCEL_SCALE) * G;
            
            // Map acceleration to notes
            mapAccelerationToNotes(accelX, accelY, accelZ);
            
            // Store current acceleration values for next iteration
            prevAcceleration[0] = accelX;
            prevAcceleration[1] = accelY;
            prevAcceleration[2] = accelZ;
        }
        
        // Sleep to avoid CPU overuse
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void MPUSynth::mapAccelerationToNotes(double x, double y, double z) {
    bool shouldPlayNote = false;
    int noteIndex = 0;
    
    // Check if any axis exceeds threshold and map to note
    if (axisEnabled[0] && shouldTriggerNote(x, prevAcceleration[0], accelerationThresholds[0])) {
        double normalizedX = std::abs(x) / 10.0; // Normalize to 0-1 range (assuming max 10G)
        noteIndex = std::min(static_cast<int>(normalizedX * axisNoteSpread[0]), axisNoteSpread[0] - 1);
        shouldPlayNote = true;
    }
    else if (axisEnabled[1] && shouldTriggerNote(y, prevAcceleration[1], accelerationThresholds[1])) {
        double normalizedY = std::abs(y) / 10.0;
        noteIndex = std::min(static_cast<int>(normalizedY * axisNoteSpread[1]), axisNoteSpread[1] - 1);
        shouldPlayNote = true;
    }
    else if (axisEnabled[2] && shouldTriggerNote(z, prevAcceleration[2], accelerationThresholds[2])) {
        double normalizedZ = std::abs(z) / 10.0;
        noteIndex = std::min(static_cast<int>(normalizedZ * axisNoteSpread[2]), axisNoteSpread[2] - 1);
        shouldPlayNote = true;
    }
    
    // Trigger note if needed
    if (shouldPlayNote && noteIndex < noteFrequencies.size()) {
        // Set the new note frequency
        currentFrequency = noteFrequencies[noteIndex];
        
        // Start a new note
        noteActive = true;
        noteTime = 0.0f;
        
        // Debug output
        std::cout << "Playing note: " << noteIndex << " (" << currentFrequency << " Hz)" << std::endl;
    }
}

bool MPUSynth::shouldTriggerNote(double acceleration, double prevAcceleration, double threshold) {
    // Check for significant change in acceleration (zero crossing with threshold)
    return (prevAcceleration <= -threshold && acceleration >= threshold) || 
           (prevAcceleration >= threshold && acceleration <= -threshold);
}