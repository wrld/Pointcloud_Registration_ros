#include "template_match.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "template_match");

  template_match match;
  match.mainloop();
  return 0;
}
