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
      auto _builder = Gtk::Builder::create();

      try
      {
        _builder->add_from_file("install/robotem_rovne_gui/share/robotem_rovne_gui/glade/robotem_rovne_gui.glade");
      }
      catch(const Glib::FileError& ex)
      {
        RCLCPP_ERROR(get_logger(), "FileError: %s", ex.what().c_str());
        rclcpp::shutdown();
      }
      catch(const Glib::MarkupError& ex)
      {
        RCLCPP_ERROR(get_logger(), "MarkupError: %s", ex.what().c_str());
        rclcpp::shutdown();
      }
      catch(const Gtk::BuilderError& ex)
      {
        RCLCPP_ERROR(get_logger(), "BuilderError: %s", ex.what().c_str());
        rclcpp::shutdown();
      }

      Gtk::Button * btn_start = 0;
      _builder->get_widget("btn_start", btn_start);
      btn_start->signal_clicked().connect(sigc::mem_fun(*this, &Node::btn_start_pressed));

      Gtk::Window * window = nullptr;
      _builder->get_widget("robotem_rovne_gui_window", window);
      return app->run(*window);
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
