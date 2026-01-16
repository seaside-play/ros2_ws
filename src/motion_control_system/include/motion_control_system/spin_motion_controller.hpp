#ifndef SPIN_MOTION_CONTROLLER_HPP_
#define SPIN_MOTION_CONTROLLER_HPP_

#include "motion_control_interface.hpp"

namespace motion_control_system {

class SpinMotionController : public MotionController {
public:
    void start() override;
    void stop() override;
    
}; // class SpinMotionController

} // namespace motion_control_system

#endif // SPIN_MOTION_CONTROLLER_HPP_