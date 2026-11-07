#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "buffered_footprint_planner/SetObjectPose.h"

namespace {

class RawTerminal {
 public:
  bool Enable() {
    fd_ = open("/dev/tty", O_RDWR);
    if (fd_ < 0 || tcgetattr(fd_, &original_) != 0) {
      return false;
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    enabled_ = tcsetattr(fd_, TCSANOW, &raw) == 0;
    return enabled_;
  }

  bool Read(char* input) const { return read(fd_, input, 1) == 1; }

  ~RawTerminal() {
    if (enabled_) tcsetattr(fd_, TCSANOW, &original_);
    if (fd_ >= 0) close(fd_);
  }

 private:
  termios original_{};
  bool enabled_ = false;
  int fd_ = -1;
};

class KeyboardSceneEditor {
 public:
  KeyboardSceneEditor() : private_node_("~") {}

  bool Initialize() {
    private_node_.param("editor/translation_step", translation_step_, 0.01);
    private_node_.param("editor/rotation_step", rotation_step_, 0.0872664626);
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    client_ = private_node_.serviceClient<buffered_footprint_planner::SetObjectPose>(
        "/simulation/set_object_pose");
    if (!client_.waitForExistence(ros::Duration(10.0))) {
      ROS_ERROR("Scene service is not available");
      return false;
    }
    locked_subscriber_ = private_node_.subscribe(
        "/simulation/start_locked", 1,
        &KeyboardSceneEditor::LockedCallback, this);
    return SelectObject("box_1");
  }

  void Run() {
    RawTerminal terminal;
    if (!terminal.Enable()) {
      ROS_ERROR("keyboard_scene_editor requires an interactive terminal");
      return;
    }
    ros::AsyncSpinner spinner(1);
    spinner.start();
    PrintHelp();
    PrintCurrent();
    while (ros::ok()) {
      char input = 0;
      if (!terminal.Read(&input)) continue;
      if (input == '\n' || input == '\r') {
        FinishEditing();
        continue;
      }
      const char key = static_cast<char>(std::toupper(
          static_cast<unsigned char>(input)));
      if (key == 'Q') break;
      if (key >= '1' && key <= '3') {
        SelectObject(std::string("box_") + key);
      } else if (key == 'V') {
        SelectObject(vehicle_id_);
      } else if (key == 'P') {
        PrintScene();
      } else if (key == 'W' || key == 'A' || key == 'S' || key == 'D' ||
                 key == 'R' || key == 'F') {
        Move(key);
      }
    }
    std::cout << "\nKeyboard editor exited; ROS simulation remains running.\n";
  }

 private:
  bool Call(std::uint8_t action, const std::string& object_id,
            double x, double y, double yaw,
            buffered_footprint_planner::SetObjectPose::Response* response) {
    buffered_footprint_planner::SetObjectPose service;
    service.request.action = action;
    service.request.object_id = object_id;
    service.request.x = x;
    service.request.y = y;
    service.request.yaw = yaw;
    if (!client_.call(service)) {
      std::cout << "\nService call failed.\n";
      return false;
    }
    *response = service.response;
    return true;
  }

  bool SelectObject(const std::string& object_id) {
    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Call(buffered_footprint_planner::SetObjectPoseRequest::QUERY_POSE,
              object_id, 0.0, 0.0, 0.0, &response) || !response.success) {
      std::cout << "\nCannot select " << object_id << ": "
                << response.message << "\n";
      return false;
    }
    object_id_ = object_id;
    x_ = response.x;
    y_ = response.y;
    yaw_ = response.yaw;
    PrintCurrent();
    return true;
  }

  void Move(char key) {
    double x = x_;
    double y = y_;
    double yaw = yaw_;
    if (key == 'W') y += translation_step_;
    if (key == 'S') y -= translation_step_;
    if (key == 'A') x -= translation_step_;
    if (key == 'D') x += translation_step_;
    if (key == 'R') yaw += rotation_step_;
    if (key == 'F') yaw -= rotation_step_;

    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Call(buffered_footprint_planner::SetObjectPoseRequest::SET_POSE,
              object_id_, x, y, yaw, &response)) return;
    if (!response.success) {
      std::cout << "\nRejected: " << response.message << "\n";
      PrintCurrent();
      return;
    }
    x_ = response.x;
    y_ = response.y;
    yaw_ = response.yaw;
    PrintCurrent();
  }

  void FinishEditing() {
    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Call(buffered_footprint_planner::SetObjectPoseRequest::FINISH_EDITING,
              object_id_, x_, y_, yaw_, &response)) return;
    std::cout << "\n" << (response.success ? "OK: " : "Rejected: ")
              << response.message << "\n";
  }

  void PrintScene() {
    const std::string ids[] = {"box_1", "box_2", "box_3", vehicle_id_};
    std::cout << "\nCurrent scene:\n";
    for (const auto& id : ids) {
      buffered_footprint_planner::SetObjectPose::Response response;
      if (Call(buffered_footprint_planner::SetObjectPoseRequest::QUERY_POSE,
               id, 0.0, 0.0, 0.0, &response) && response.success) {
        std::cout << "  " << id << ": x=" << response.x
                  << " y=" << response.y
                  << " yaw=" << response.yaw * 180.0 / M_PI << " deg\n";
      }
    }
    PrintCurrent();
  }

  void PrintHelp() const {
    std::cout << "\nW/S: +Y/-Y  A/D: -X/+X  R/F: CCW/CW\n"
              << "1/2/3: select obstacle  V: Vehicle 1  P: print scene\n"
              << "Enter: finish placement/start 3 s lock  Q: quit editor\n";
  }

  void PrintCurrent() const {
    std::cout << "\r[" << (object_id_ == vehicle_id_ ? "VEHICLE" : "OBSTACLE")
              << "] " << object_id_ << "  x=" << x_ << "  y=" << y_
              << "  yaw=" << yaw_ * 180.0 / M_PI << " deg          "
              << std::flush;
  }

  void LockedCallback(const std_msgs::Bool::ConstPtr& message) {
    if (message->data && !last_locked_) {
      std::cout << "\nVehicle 1 start locked. Use RViz 2D Nav Goal.\n";
      PrintCurrent();
    }
    last_locked_ = message->data;
  }

  ros::NodeHandle private_node_;
  ros::ServiceClient client_;
  ros::Subscriber locked_subscriber_;
  std::string vehicle_id_ = "vehicle_1";
  std::string object_id_ = "box_1";
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double translation_step_ = 0.01;
  double rotation_step_ = 0.0872664626;
  bool last_locked_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "keyboard_scene_editor");
  KeyboardSceneEditor editor;
  if (!editor.Initialize()) return EXIT_FAILURE;
  editor.Run();
  return EXIT_SUCCESS;
}
