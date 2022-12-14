#include "main.h"
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}


int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "comp_main_node");
    ros::NodeHandle n("~");

    khnp_comp kcp(n);

    signal(SIGINT, signal_handler); // to exit program when ctrl+c

    ros::AsyncSpinner spinner(8); // Use 10 threads -> 4 callbacks + 4 Timer callbacks
    spinner.start();
    // ros::waitForShutdown();

    return app.exec();
}