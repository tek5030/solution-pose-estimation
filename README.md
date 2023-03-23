# Pose estimation and Augmented Reality

This is our proposed solution for the lab ["Pose estimation and Augmented Reality"][repo] in the computer vision course [TEK5030] at the University of Oslo.

Please see the [lab guide][guide] for more information.


## Prerequisites

For this lab, we can unfortunately not rely on conan to install all required OpenCV modules (namely the `viz` module for 3D visualization). You have a few other options:

- Solve the [python lab](https://github.com/tek5030/lab-pose-estimation-py) (recommended)
- Use the lab computers
- Install OpenCV using [homebrew](https://brew.sh/) (option for mac and linux). (See also [Getting started on MacOS](https://tek5030.github.io/tutorial/macos.html).)
- Rely on virtualbox and our prepared linux image with dependencies preinstalled (see [Canvas: Setting up your computer for the labs](https://uio.instructure.com/courses/44675/discussion_topics/295673))
- Try [Docker toolchain][docker-toolchain] in CLion (very experimental)

[repo]:  https://github.com/tek5030/lab-pose-estimation
[guide]: https://github.com/tek5030/lab-pose-estimation/blob/master/README.md

[TEK5030]: https://www.uio.no/studier/emner/matnat/its/TEK5030/
[conan]: https://tek5030.github.io/tutorial/conan.html
[lab_intro]: https://github.com/tek5030/lab-intro/blob/master/cpp/lab-guide/1-open-project-in-clion.md#6-configure-project
[docker-toolchain]: https://tek5030.github.io/tutorial/dev-container.html