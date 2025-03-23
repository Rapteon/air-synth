#ifndef __GPIOEVENT_H
#define __GPIOEVENT_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <thread>
#include <gpiod.h>
#include <vector>

#ifndef NDEBUG
#define DEBUG
#endif

#define ISR_TIMEOUT 1 // sec

class GPIOPin {

public:
    ~GPIOPin() {
        stop();
    }

    struct GPIOEventCallbackInterface {
        virtual void hasEvent(gpiod_line_event& e) = 0;
    };

    enum PullResistor {
        NO_PULL,
        PULL_UP,
        PULL_DOWN
    };

    void registerCallback(GPIOEventCallbackInterface* ci) {
        adsCallbackInterfaces.push_back(ci);
    }

    void start(int pinNo, int chipNo = 0, PullResistor pullResistor = NO_PULL);

    void stop();

private:
    void gpioEvent(gpiod_line_event& event);

    void worker();

    gpiod_chip* chipGPIO = nullptr;
    gpiod_line* pinGPIO = nullptr;

    std::thread thr;

    bool running = false;

    std::vector<GPIOEventCallbackInterface*> adsCallbackInterfaces;
};

#endif