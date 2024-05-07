/**
 * Copyright (c) 2024 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/robotem_rovne_gui/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <robotem_rovne_gui/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace t07
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node(int & argc, char ** argv)
: rclcpp::Node("robotem_rovne_gui_node")
{
  _gui_thread = std::thread(
    [this, &argc, &argv]()
    {
      auto app = Gtk::Application::create(argc, argv, "");

      Gtk::Window window;
      window.set_title("Robotem Rovne GUI");
      window.set_default_size(320, 240);

      Gtk::Button btn_start;
      btn_start.set_label("Start");
      btn_start.signal_clicked().connect(sigc::mem_fun(*this, &Node::btn_start_pressed));

      window.add(btn_start);

      window.show_all_children();

      return app->run(window);
    });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  _gui_thread.join();
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::btn_start_pressed()
{
  RCLCPP_INFO(get_logger(), "btn_start_pressed");
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
