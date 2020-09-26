
<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/nadavleh/Chess_AI">
  </a>

  <h3 align="center">SLAM using Rao-Blawellised Particle Filter with DE-NDT and gradient NDT scan-matching</h3>

  <p align="center">
    This is simple python chess program written with pygame for the GUI. The chess AI uses minimax with alpha-beta prunning
    <br />
    <a href="https://github.com/nadavleh/Chess_AI"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/nadavleh/Chess_AI">View Demo</a>
    ·
    <a href="https://github.com/nadavleh/Chess_AI/issues">Report Bug</a>
    ·
    <a href="https://github.com/nadavleh/Chess_AI/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Usage](#usage)
* [Contributing](#contributing)
* [License](#license)



<!-- ABOUT THE PROJECT -->
## About The Project

[![Product Name Screen Shot][product-screenshot]](https://github.com/nadavleh/SLAM-Rao-Blackwellised-PF-with-DE-NDT-and-gradient-NDT-scan-matching/screenshot.png)

In this project we use the real-time wheel odometry data aswell as real-time LIDAR scans from a robot simulated in the Gazebo environment, in order to map its surroundings and localize itself within the constructed map. The technique we use here is based on the Rao-Blackwell marginalization of the posterior distribution of the state vector, which consists of the robot's location (x,y), oriantation (angle), and each of the grid's cells. This marginalization enables us to construct a map from the LIDAR readings at time t, and then to localize the robot at time t+1, within the constructed map of the previous time step (the localization uses the odometry as model propagation and the LIDAR readings for the Bayesian likelyhood update). Once the localization is done (using a particle filter), the robot may continue to build the map using the same lidar scans it has used to localize itself at time t+1, to produce un updated map at time t+1. 

However, the robot's state estimation has small errors which accumilate over time, thus causing the built map to be a poor description of the actual surroundings. To tackle this issue, every time the robot updates its state i.e. localizes, we need to verify that the LIDAR readings actually correspond to the current map. If they do not correspond well, we may start seeing straight walls turning into a few walls that cross each other. Thus we may infer thet the localization was erroneous and we may try to find the propper tranformation of the robots state at time t to the one at time t+1. This is done using what is known as Scan Matching. Scan Matching deals with the following question: what is the tranformation that best turns the LIDAR reading from time t to the readings at time t+1? this can be done using a nuber of different ways, where each has its own merrits.
Here, we use both Differential Evolution (DE) and Gradient (Newton–Raphson method for multivariable functions) optimizations (not together but seperatly, so we can compare them) of the Normal Distribution Transform (see: https://ieeexplore.ieee.org/document/1249285)


The files are interliked, make sure they are all in your MATLAB's path inorder for them to work.
### Built With

* [MATLAB](https://www.mathworks.com/products/matlab.html)


## Usage


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/nadavleh/repo.svg?style=flat-square
[forks-shield]: https://img.shields.io/github/forks/nadavleh/repo.svg?style=flat-square
[forks-url]: https://github.com/nadavleh/repo/network/members
[stars-shield]: https://img.shields.io/github/stars/nadavleh/repo.svg?style=flat-square
[stars-url]: https://github.com/nadavleh/repo/stargazers
[issues-shield]: https://img.shields.io/github/issues/nadavleh/repo.svg?style=flat-square
[issues-url]: https://github.com/nadavleh/repo/issues
[license-shield]: https://img.shields.io/github/license/nadavleh/repo.svg?style=flat-square
[product-screenshot]: https://github.com/nadavleh/Chess_AI/blob/master/images/screenshot.png

