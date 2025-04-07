#include "Button/Button.h"
#include <cstdio>
#include <thread>

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
        throw "GPIO chip error.\n";
    }

    line = gpiod_chip_get_line(chip, offset);
    if (NULL == line)
    {
        throw "GPIO line error.\n";
    }

    // int result = gpiod_line_request(line, &LINE_CONFIG, 0);
    int result = gpiod_line_request_both_edges_events(line, "Consumer");
    if (result < 0)
    {
        throw "Could not request line for IRQ.";
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
            // TODO: Handle error condition
            break;
        case 0:
            // TODO: Handle timeout condition
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