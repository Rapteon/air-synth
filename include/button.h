#ifndef BUTTON_H
#define BUTTON_H

#include <gpiod.h>
#include <thread>
#include <vector>

class Button
{
public:
    Button();
    ~Button() {
        stop();
    }

    struct ButtonCallbackInterface
    {
        /**
         * Called when a new sample is available.
         * This needs to be implemented in a derived
         * class by the client. Defined as abstract.
         * \param e If falling or rising.
         **/
        virtual void hasEvent(gpiod_line_event &e) = 0;
    };

    void registerCallback(ButtonCallbackInterface *ci)
    {
        registeredCallbacks.push_back(ci);
    }

    void start(int pinNo,
               int ChipNo = 0);

    void stop();

private:
    void onEvent(gpiod_line_event &event);

    void worker();

    gpiod_chip *chipGPIO = nullptr;
    gpiod_line *pinGPIO = nullptr;

    // thread
    std::thread thr;

    // flag that it's running
    bool running = false;

    std::vector<ButtonCallbackInterface *> registeredCallbacks;
}

#endif