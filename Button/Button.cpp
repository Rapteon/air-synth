#include "Button/Button.h"
#include <iostream>
#include <thread>
#include <stdexcept>
#include <cstring>
#include <errno.h>

Button::~Button()
{
    stop();
}

void Button::registerCallback(ButtonCallbackInterface *ci)
{
    registeredCallbacks.push_back(ci);
}

void Button::start(const char *chipName, int offset)
{
    chip = gpiod_chip_open_by_name(chipName);
    if (NULL == chip)
    {
        std::string errorMsg = "GPIO chip error: ";
        errorMsg += std::strerror(errno);
        throw std::runtime_error(errorMsg);
    }

    line = gpiod_chip_get_line(chip, offset);
    if (NULL == line)
    {
        std::string errorMsg = "GPIO line error: ";
        errorMsg += std::strerror(errno);
        gpiod_chip_close(chip);
        throw std::runtime_error(errorMsg);
    }

    int result = gpiod_line_request_both_edges_events(line, "Consumer");
    if (result < 0)
    {
        std::string errorMsg = "Could not request line for IRQ: ";
        errorMsg += std::strerror(errno);
        errorMsg += ". This may be due to insufficient permissions. Try running with sudo.";
        gpiod_line_release(line);
        gpiod_chip_close(chip);
        throw std::runtime_error(errorMsg);
    }
    running = true;

    workerThread = std::thread(&Button::worker, this);
}

void Button::stop()
{
    if (!running)
        return;

    workerThread.join();
    gpiod_line_release(line);
    gpiod_chip_close(chip);
}

void Button::onEvent(gpiod_line_event &event)
{
    for (auto &cb : registeredCallbacks)
    {
        cb->hasEvent(event);
    }
}

void Button::worker()
{
    while (running)
    {
        int result = gpiod_line_event_wait(line, nullptr);
        switch (result)
        {
        case -1:
            fprintf(stderr, "GPIOD error occurred while waiting for event.\n");
            break;
        case 0:
            fprintf(stderr, "Timed out while waiting for GPIOD event.\n");
            break;
        case 1:
            fprintf(stderr, "GPIO Event case arm.\n");
            gpiod_line_event event;
            gpiod_line_event_read(line, &event);
            onEvent(event);
            break;
        }
    }
    gpiod_chip_close(chip);
}