#include <gpiod.h>
#include "GPIOEVENT.h"

void GPIOPin::start(int pinNo, int chipNo, PullResistor pullResistor) {
#ifdef DEBUG
    fprintf(stderr, "Init.\n");
#endif

    chipGPIO = gpiod_chip_open_by_number(chipNo);
    if (NULL == chipGPIO) {
#ifdef DEBUG
        fprintf(stderr, "GPIO chip could not be accessed.\n");
#endif
        throw "GPIO chip error.\n";
    }

    pinGPIO = gpiod_chip_get_line(chipGPIO, pinNo);
    if (NULL == pinGPIO) {
#ifdef DEBUG
        fprintf(stderr, "GPIO line could not be accessed.\n");
#endif
        throw "GPIO line error.\n";
    }

    struct gpiod_line_request_config config;
    config.consumer = "Consumer";
    config.request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;

    if (pullResistor == PULL_UP) {
        config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else if (pullResistor == PULL_DOWN) {
        config.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    } else {
        config.flags = 0;
    }

    int ret = gpiod_line_request(pinGPIO, &config, 0);

    if (ret < 0) {
#ifdef DEBUG
        fprintf(stderr, "Request event notification failed on pin %d and chip %d.\n", pinNo, chipNo);
#endif
        throw "Could not request event for IRQ.";
    }

    running = true;

    thr = std::thread(&GPIOPin::worker, this);
}

void GPIOPin::gpioEvent(gpiod_line_event& event) {
    for (auto& cb : adsCallbackInterfaces) {
        cb->hasEvent(event);
    }
}

void GPIOPin::worker() {
    while (running) {
        const timespec ts = { ISR_TIMEOUT, 0 };
        int r = gpiod_line_event_wait(pinGPIO, &ts);
        if (1 == r) {
            gpiod_line_event event;
            gpiod_line_event_read(pinGPIO, &event);
            gpioEvent(event);
        } else if (r < 0) {
#ifdef DEBUG
            fprintf(stderr, "GPIO error while waiting for event.\n");
#endif
        }
    }
}

void GPIOPin::stop() {
    if (!running) return;
    running = false;
    thr.join();
    gpiod_line_release(pinGPIO);
    gpiod_chip_close(chipGPIO);
}