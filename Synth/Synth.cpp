#include "Synth.h"
#include "mpu6050.h"
#include "ControllerEvent/ControllerEvent.h"
#include <stdexcept>

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
      noteDuration(250.0f),
      noteActive(false),
      currentFrequency(440.0f),
      noteAmplitude(0.0f),
      noteTime(0.0f),
      running(false),
      continuousMode(true)
{
    // Initialize STK
    try {
        stk::Stk::setSampleRate(sampleRate);
        std::cout << "STK sample rate set to " << sampleRate << std::endl;

        std::string rawwavePath = "/usr/share/stk/rawwaves/";
        stk::Stk::setRawwavePath(rawwavePath);
        std::cout << "STK rawwave path set to: " << rawwavePath << std::endl;
        
        // Default instrument is a piano (Rhodey)
        instrument = std::make_unique<stk::Rhodey>();
        std::cout << "STK instrument created successfully" << std::endl;
        
        // Test the instrument
        instrument->noteOn(440.0, 0.5);
        std::cout << "STK note triggered" << std::endl;
    } catch (const stk::StkError& e) {
        // Create a non-const copy to get the message
        stk::StkError error = e;
        throw std::runtime_error("STK initialization failed: " + std::string(error.getMessage()));
    }
    
    // Default scale is C major
    generateNotes("C", ScaleType::MAJOR);
    
    // Default mapping settings
    axisEnabled = {true, true, true};
    axisNoteSpread = {8, 8, 8};
    axisVelocitySensitivity = {1.0f, 1.0f, 1.0f};
    
    // Default acceleration thresholds
    accelerationThresholds = {3.0, 3.0, 2.0};
    
    // Initialize previous acceleration values
    prevAcceleration = {0.0, 0.0, 0.0};
}

MPUSynth::~MPUSynth() {
    stop();
}

void MPUSynth::start() {
    if (running) return;

    // Initialize RtAudio
    RtAudio::StreamParameters parameters;
    parameters.deviceId = dac.getDefaultOutputDevice();
    parameters.deviceId = 0;
    parameters.nChannels = 2;
    parameters.firstChannel = 0;
    
    unsigned int bufferFrames = 1024;
    volume = 1.0f;

    try {
        dac.openStream(&parameters, nullptr, RTAUDIO_FLOAT32,
                       sampleRate, &bufferFrames, &MPUSynth::audioCallback, this);
        dac.startStream();
    }
    catch (const RtAudioError& e) {
        throw std::runtime_error("RtAudio initialization failed: " + std::string(e.getMessage()));
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
        catch (const RtAudioError& e) {
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
    
    try {
        switch (instrumentType) {
            case InstrumentType::PIANO:
            case InstrumentType::RHODEY:
                instrument = std::make_unique<stk::Rhodey>();
                break;
            case InstrumentType::WURLEY:
                instrument = std::make_unique<stk::Wurley>();
                break;
            case InstrumentType::MOOG:
                instrument = std::make_unique<stk::Moog>();
                break;
            case InstrumentType::CLARINET:
                instrument = std::make_unique<stk::Clarinet>();
                break;
        }
        
        std::cout << "Instrument created successfully" << std::endl;
    } catch (const stk::StkError& e) {
        // Create a non-const copy to get the message
        stk::StkError error = e;
        throw std::runtime_error("Failed to create instrument: " + std::string(error.getMessage()));
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

int MPUSynth::noteToMidiNumber(const std::string& note) const {
    auto it = NOTE_TO_MIDI.find(note);
    if (it != NOTE_TO_MIDI.end()) {
        return it->second;
    }
    return -1; // Note not found
}

float MPUSynth::midiNumberToFrequency(int midiNumber) const {
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

    // If instrument is null, use fallback sine wave generation
    if (!instrument) {
        // Simple sine wave generation
        static float phase = 0.0f;
        
        for (unsigned int i = 0; i < nBufferFrames; i++) {
            float sample = 0.0f;
            
            if (noteActive.load()) {
                // Generate simple sine wave
                sample = 0.8f * volume.load() * sin(phase);
                
                // Update phase
                phase += 2.0f * M_PI * currentFrequency.load() / sampleRate;
                if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;
                
                // Increment note time
                float currentTime = noteTime.load();
                noteTime.store(currentTime + (1000.0f / sampleRate));
                
                // Check if note is finished
                if (noteTime.load() >= noteDuration.load()) {
                    noteActive.store(false);
                }
            }
            
            // Output to both channels (stereo)
            buffer[i*2] = sample;      // Left channel
            buffer[i*2+1] = sample;    // Right channel
        }
        
        return;
    }
    
    static int frameCounter = 0;
    
    for (unsigned int i = 0; i < nBufferFrames; i++) {
        float sample = 0.0f;
        
        if (noteActive.load()) {
            // Calculate amplitude envelope
            if (noteTime.load() < 10.0f) {
                // Attack phase
                noteAmplitude.store((noteTime.load() / 10.0f) * volume.load());
            } else if (noteTime.load() > (noteDuration.load() - 50.0f)) {
                // Release phase
                float releasePhase = (noteDuration.load() - noteTime.load()) / 50.0f;
                noteAmplitude.store(std::max(0.0f, releasePhase * volume.load()));
            } else {
                // Sustain phase
                noteAmplitude.store(volume.load());
            }
            
            // Get raw sample from STK
            sample = instrument->tick() * noteAmplitude.load();
            
            // Update note time
            float currentTime = noteTime.load();
            noteTime.store(currentTime + (1000.0f / sampleRate));
            
            // Check if note is finished
            if (noteTime.load() >= noteDuration.load()) {
                noteActive.store(false);
                instrument->noteOff(0.5); // Send explicit note off
            }
        } else {
            // Get any residual sound from the instrument
            sample = instrument->tick() * 0.001f; // Very low volume for release tail
        }
        
        // Apply to both channels
        buffer[i*2] = sample;
        buffer[i*2+1] = sample;
    }
    
    // Log occasionally
    if (++frameCounter % 1000 == 0) {
        float rawSample = instrument->tick();
        std::cout << "Audio frame " << frameCounter 
                  << ": noteActive=" << (noteActive.load() ? "true" : "false")
                  << ", freq=" << currentFrequency.load() 
                  << ", amp=" << noteAmplitude.load()
                  << ", rawSample=" << rawSample << std::endl;
    }
}

void MPUSynth::addEvent(const ControllerEvent& event) {
    std::lock_guard<std::mutex> lock(queueMutex);
    eventQueue.push(event);
}

void MPUSynth::processMpuData() {
    constexpr int MAX_QUEUE_SIZE = 5;
    
    while (running.load()) {
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (!eventQueue.empty()) {
                // Process the most recent event in the queue
                ControllerEvent e = eventQueue.front();
                double x = e.getX();
                double y = e.getY();
                double z = e.getZ();

                mapAccelerationToNotes(x, y, z);

                prevAcceleration[0] = x;
                prevAcceleration[1] = y;
                prevAcceleration[2] = z;

                // Remove processed event
                eventQueue.pop();
                
                // If queue is getting too full, clear out older events
                while (eventQueue.size() > MAX_QUEUE_SIZE) {
                    eventQueue.pop();
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void MPUSynth::mapAccelerationToNotes(double x, double y, double z) {
    // Calculate the magnitude of acceleration for each axis
    double absX = std::abs(x);
    double absY = std::abs(y);
    double absZ = std::abs(z);
    
    // Calculate total acceleration magnitude
    double totalAccel = std::sqrt(x*x + y*y + z*z);
    
    // Get the direction of movement (positive or negative)
    // This helps differentiate between different types of motion
    int dirX = (x > 0) ? 1 : -1;
    int dirY = (y > 0) ? 1 : -1;
    int dirZ = (z > 0) ? 1 : -1;
    
    // Normalize acceleration values with more dynamic range
    // Reduce the divisor to make it more sensitive to smaller movements
    double normX = std::min(absX / 10.0, 1.0) * dirX;
    double normY = std::min(absY / 10.0, 1.0) * dirY;
    double normZ = std::min(absZ / 8.0, 1.0) * dirZ;
    
    // Calculate discrete note indices for each axis
    // This creates more distinct notes rather than continuous blending
    int indexX = 0, indexY = 0, indexZ = 0;
    
    if (axisEnabled[0]) {
        // Map X-axis motion to notes in the first third of the scale
        indexX = std::min(static_cast<int>(std::abs(normX) * axisNoteSpread[0]), 
                         axisNoteSpread[0] - 1);
        // Shift index based on direction to use different parts of the scale
        if (x < 0) indexX += axisNoteSpread[0];
    }
    
    if (axisEnabled[1]) {
        // Map Y-axis motion to notes in the middle third of the scale
        indexY = std::min(static_cast<int>(std::abs(normY) * axisNoteSpread[1]), 
                         axisNoteSpread[1] - 1);
        if (y < 0) indexY += axisNoteSpread[1];
    }
    
    if (axisEnabled[2]) {
        // Map Z-axis to higher notes
        indexZ = std::min(static_cast<int>(std::abs(normZ) * axisNoteSpread[2]), 
                         axisNoteSpread[2] - 1);
        if (z < 0) indexZ += axisNoteSpread[2];
    }
    
    // Determine which axis had the strongest motion
    double maxMotion = std::max({absX * axisVelocitySensitivity[0], 
                                absY * axisVelocitySensitivity[1],
                                absZ * axisVelocitySensitivity[2]});
    
    // Select note index based on the dominant axis
    int selectedIndex = 0;
    if (axisEnabled[0] && absX * axisVelocitySensitivity[0] >= maxMotion * 0.8) {
        selectedIndex = indexX;
        std::cout << "X-dominant motion: index " << indexX << std::endl;
    } else if (axisEnabled[1] && absY * axisVelocitySensitivity[1] >= maxMotion * 0.8) {
        selectedIndex = indexY;
        std::cout << "Y-dominant motion: index " << indexY << std::endl;
    } else if (axisEnabled[2] && absZ * axisVelocitySensitivity[2] >= maxMotion * 0.8) {
        selectedIndex = indexZ;
        std::cout << "Z-dominant motion: index " << indexZ << std::endl;
    } else {
        // If no dominant axis, use a weighted average
        int totalNotes = noteFrequencies.size() - 1;
        double weightedSum = 0.0;
        double totalWeight = 0.0;
        
        if (axisEnabled[0] && absX > 0.1) {
            weightedSum += indexX * (absX * axisVelocitySensitivity[0]);
            totalWeight += absX * axisVelocitySensitivity[0];
        }
        
        if (axisEnabled[1] && absY > 0.1) {
            weightedSum += indexY * (absY * axisVelocitySensitivity[1]);
            totalWeight += absY * axisVelocitySensitivity[1];
        }
        
        if (axisEnabled[2] && absZ > 0.1) {
            weightedSum += indexZ * (absZ * axisVelocitySensitivity[2]);
            totalWeight += absZ * axisVelocitySensitivity[2];
        }
        
        if (totalWeight > 0.1) {
            selectedIndex = static_cast<int>(weightedSum / totalWeight) % totalNotes;
        } else {
            // If very little motion, keep current note
            selectedIndex = static_cast<int>((currentFrequency.load() - noteFrequencies[0]) / 
                           (noteFrequencies[1] - noteFrequencies[0]));
            selectedIndex = std::min(std::max(0, selectedIndex), totalNotes);
        }
        std::cout << "Combined motion: index " << selectedIndex << std::endl;
    }
    
    // Ensure index is within bounds
    selectedIndex = std::min(selectedIndex, static_cast<int>(noteFrequencies.size() - 1));
    selectedIndex = std::max(0, selectedIndex);
    
    // Get the new frequency
    float newFrequency = noteFrequencies[selectedIndex];
    
    // Calculate velocity based on total acceleration with higher sensitivity
    float velocity = std::min(1.0f, static_cast<float>(totalAccel / 15.0));
    velocity = std::max(0.3f, velocity); // Ensure minimum velocity
    
    // Check if we should trigger a new note
    bool shouldTriggerNew = false;
    
    // If frequency changed significantly or no note is playing
    if (!noteActive.load() || std::abs(newFrequency - currentFrequency.load()) > 0.5) {
        shouldTriggerNew = true;
    }
    
    // If motion exceeds threshold, always trigger a new note
    if (totalAccel > 4.0) {
        shouldTriggerNew = true;
    }
    
    if (shouldTriggerNew) {
        // Set the new note frequency
        currentFrequency.store(newFrequency);
        
        // Reset the instrument and trigger a note
        if (instrument) {
            std::lock_guard<std::mutex> lock(instrumentMutex);
            
            // Turn off previous note
            instrument->noteOff(0.5);
            
            // Set new frequency
            instrument->setFrequency(currentFrequency.load());
            
            // Trigger note with calculated velocity
            instrument->noteOn(currentFrequency.load(), velocity);
            
            std::cout << "New note: index=" << selectedIndex 
                     << ", frequency=" << currentFrequency.load() 
                     << " Hz, Velocity: " << velocity 
                     << ", Total acceleration: " << totalAccel << std::endl;
        }
        
        // Start new note envelope
        noteActive.store(true);
        noteAmplitude.store(0.0f);
        noteTime.store(0.0f);
    } else if (noteActive.load()) {
        // Update velocity of current note if significant change
        if (std::abs(velocity - noteAmplitude.load()) > 0.1) {
            if (instrument) {
                std::lock_guard<std::mutex> lock(instrumentMutex);
                instrument->controlChange(128, static_cast<int>(velocity * 127.0f));
            }
            noteAmplitude.store(velocity);
        }
    }
}

bool MPUSynth::shouldTriggerNote(double acceleration, double prevAcceleration, double threshold) const {
    double diff = std::abs(acceleration - prevAcceleration);
    return diff >= threshold;
}