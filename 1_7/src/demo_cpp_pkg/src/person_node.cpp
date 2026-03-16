#include <string>
#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node {
private:
    std::string name_;
    int age_;

public:
    PersonNode(const std::string& node_name, const std::string& name, int age)
        : Node(node_name) {
            name_ = name;
            age_ = age;
        }
    
    void eat(const std::string& food_name) {
        RCLCPP_INFO(this->get_logger(), 
            "I'm  %s, it's %d years old, I'm eating %s", name_.c_str(), age_, food_name.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>("cpp_node", "吴潮江", 18);
    RCLCPP_INFO(node->get_logger(), "你好 C++节点");
    node->eat("饺子");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// colcon build，其实是调用cmake和make