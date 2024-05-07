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

Node::Node()
: rclcpp::Node("robotem_rovne_gui_node")
, _gui_thread{}
{
  init_glut();
  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  deinit_glut();
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_glut()
{
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

  glutInitWindowPosition(50, 50);
  glutInitWindowSize(320, 240);

  glutCreateWindow("Robotem Rovne GUI");

  auto const glut_render = +[]()
  {
    /* Just clear the buffers for now. */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    /* Flip the buffers. */
    glutSwapBuffers();
  };

  glutDisplayFunc(glut_render);
  glutIdleFunc(glut_render);

  _gui_thread = std::thread([this]() { glutMainLoop(); });
}

void Node::deinit_glut()
{
  glutLeaveMainLoop();
  _gui_thread.join();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
