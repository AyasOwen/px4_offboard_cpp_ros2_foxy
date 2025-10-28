#ifndef TI_2024_HPP
#define TI_2024_HPP

#include "offboard_control_node.hpp"


class OffboardNode_2024 : public OffboardControlNode {
public:
    OffboardNode_2024();

protected:
    void control_loop() override;
    void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};

#endif // TI_2024_HPP
