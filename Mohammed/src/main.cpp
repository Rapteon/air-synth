/**
 * @file main.cpp
 * @brief Entry point for the Air Synth application
 * @author Mohammed
 * @date Feb 25, 2025
 */

#include "MPU6050.hpp"
#include "GestureDetector.hpp"
#include "Synthesizer.hpp"
#include "AirSynth.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstdlib>

// Global flag for program control
volatile sig_atomic_t g_running = 1;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    g_running = 0;
}

int main(int argc, char* argv[]) {
    std::cout << "==================================" << std::endl;
    std::cout << "Air Synth - MPU6050 Motion Synthesizer" << std::endl;
    std::cout << "==================================" << std::endl;
    
    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Create the Air Synth application
        AirSynth airSynth;
        
        // Initialize the application
        if (!airSynth.initialize()) {
            std::cerr << "Failed to initialize Air Synth application. Exiting." << std::endl;
            return EXIT_FAILURE;
        }
        
        std::cout << "Air Synth initialized. Starting main loop..." << std::endl;
        
        // Main application loop
        while (g_running) {
            // Update the Air Synth state
            airSynth.update();
            
            // Small delay to avoid hogging CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Shutdown the application
        std::cout << "Shutting down Air Synth..." << std::endl;
        airSynth.shutdown();
        
        std::cout << "Air Synth shutdown complete." << std::endl;
        return EXIT_SUCCESS;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return EXIT_FAILURE;
    }
}