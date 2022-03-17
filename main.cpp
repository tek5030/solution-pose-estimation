#include "solution_pose_estimation.h"
#include <iostream>

int main()
{
  try
  {
    runPoseEstimationSolution();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Caught exception:" << std::endl
              << e.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << "Caught unknown exception" << std::endl;
  }

  return EXIT_SUCCESS;
}
