# Patchwork++

## :bookmark_tabs: About Patchwork++ (IROS'22)

* A fast, robust, and self-adaptive **ground segmentation algorithm** on 3D point cloud.

<p align="center"><img src=pictures/pcap-demo.gif alt="animated" /></p>

* An extension of [Patchwork][patchworklink] (RA-L'21 with IROS'21).
* Please refer our [paper][patchworkppIEEElink] for detailed explanantions and experimental results!

   * Validated on [SemanticKITTI][SemanticKITTIlink] dataset. The benchmark code is available on [here][benchmarklink].

* :bulb: Contents: [YouTube][YouTubeLink], [arXiv][arXivlink], [IEEE *Xplore*][patchworkppIEEElink]

[YouTubeLInk]: https://www.youtube.com/watch?v=fogCM159GRk
[arXivlink]: https://arxiv.org/abs/2207.11919
[patchworklink]: https://github.com/LimHyungTae/patchwork
[SemanticKITTIlink]: http://www.semantic-kitti.org/
[benchmarklink]: https://github.com/url-kaist/Ground-Segmentation-Benchmark

## :open_file_folder: What's in this repo

* C++ source code of Patchwork++ ([patchworkpp][sourcecodelink])
* Python binding of Patchwork++ using pybind11 ([python_wrapper][wraplink])
* Examples codes, which visualizes a ground segmentation result by Patchwork++ ([examples][examplelink]) :thumbsup:

> If you are familiar with ROS, you can also visit [here][roslink] and try executing ROS-based Patchwork++!

[roslink]: https://github.com/url-kaist/patchwork-plusplus-ros

[sourcecodelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/patchworkpp
[pybind11link]: https://github.com/pybind/pybind11
[wraplink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/python_wrapper
[examplelink]: https://github.com/url-kaist/patchwork-plusplus/tree/master/examples

## :package: Prerequisite packages
> You may need to install Eigen, numpy, and Open3D. Open3D is used for point cloud visualization.

```bash
# To install Eigen and numpy
$ sudo apt-get install libeigen3-dev
$ pip install numpy

# To install Open3D Python packages
$ pip install open3d

# To install Open3D C++ packages
$ git clone https://github.com/isl-org/Open3D
$ cd Open3D
$ util/install_deps_ubuntu.sh # Only needed for Ubuntu
$ mkdir build && cd build
$ cmake ..
$ make
$ sudo make install
```

## :gear: How to build
> Please follow below codes to build Patchwork++.

```bash
$ git clone https://github.com/url-kaist/patchwork-plusplus
$ cd patchwork-plusplus
$ mkdir build && cd build
$ cmake ..
$ make
```

## :runner: To run the demo codes
> There are some example codes for your convenience!
> Please try using Patchwork++ to segment ground points in a 3D point cloud :smiley:

### Python
```bash
# Run patchwork++ with sequential point cloud inputs 
$ python examples/python/demo_sequential.py
```
