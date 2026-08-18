#pragma once

class EventResponder;
using EventResponderRef = EventResponder&;

class EventResponder {
public:
    void clearEvent() { triggered_ = false; }
    void triggerEvent(int = 0, void* = nullptr) { triggered_ = true; }
    explicit operator bool() const { return triggered_; }

private:
    bool triggered_ = false;
};
