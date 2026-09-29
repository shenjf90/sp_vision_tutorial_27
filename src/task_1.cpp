#include <opencv2/opencv.hpp>

#include "task_common.hpp"

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, "{help h usage ? | | 帮助}{@config-path | | YAML 配置文件}");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }
  return tutorial::run(tutorial::Task::detection, cli.get<std::string>("@config-path"));
}
