#ifndef BUTTON_H
#define BUTTON_H

#include <gpiod.h>
#include <thread>
#include <vector>


class Button
{
public:
    ~Button();

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

    /**
     * Registers callback handlers which would be called
     * upon a GPIO event.
     * Callback handler classes must implement the ButtonCallbackInterface.
     */
    void registerCallback(ButtonCallbackInterface *ci);

    /**
     * Starts listening for events from the specified GPIO chip,
     * and to obtain a line at a specific offset.
     */
    void start(const char *chipName, int offset = 20);

    /**
     * Releases line and closes chip
     * after worker stops running.
     */
    void stop();

private:
    /**
     * Notifies registered callbacks when a GPIO event
     * occurs.
     */
    void onEvent(gpiod_line_event &event);

    /**
     * Blocks until a GPIO event occurs on the line.
     */
    void worker();

    gpiod_chip *chip = nullptr;
    gpiod_line *line = nullptr;
    // gpiod_line_request_config LINE_CONFIG{"button_monitor", GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP};
    gpiod_line_request_config LINE_CONFIG{"button_monitor", GPIOD_LINE_REQUEST_EVENT_RISING_EDGE, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP};

    // thread
    std::thread workerThread;

    // flag that it's running
    bool running = false;

    std::vector<ButtonCallbackInterface *> registeredCallbacks;
};

#endif
