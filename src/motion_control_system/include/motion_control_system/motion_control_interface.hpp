#ifndef MOTION_CONTROL_INTERFACE_HPP_
#define MOTION_CONTROL_INTERFACE_HPP_

namespace motion_control_system {

class MotionController {
public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual ~MotionController() {}
    
}; // class MotionController

} // namespace motion_control_system

#endif // MOTION_CONTROL_INTERFACE_HPP_