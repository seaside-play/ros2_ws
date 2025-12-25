import rclpy
from rclpy.node import Node

class PersonNode(Node):
  def __init__(self, node_name: str, name: str, age: int) -> Node: 
    super().__init__(node_name)
    self.name = name
    self.age = age
  
  def eat(self, food_name: str):
    # using {} to use variable
    print(f'I am {self.name}, {self.age} years old, I am eating {food_name}')

def main(args=None):
  rclpy.init(args=args)
  node = PersonNode('person_node1', 'chris', 43)
  node.eat('noodle')
  node.get_logger().info('I have already eat noodle')
  rclpy.spin(node)
  rclpy.shutdown()
