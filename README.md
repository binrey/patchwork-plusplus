# Python-визуализация Patchwork++ на примере записи облака точек velodyne (PCAP-файл)

<p align="center"><img src=pictures/pcap-demo.gif alt="animated" /></p>

Полное видео демонстрации: [yandex-disk][demolink]

[demolink]: https://disk.yandex.ru/i/4CBokKOJLV1C-w

## :package: Prerequisite packages
> You may need to install Eigen, numpy, velodyne_decoder and Open3D. Open3D is used for point cloud visualization.

```bash
# To install Eigen and numpy
$ sudo apt-get install libeigen3-dev
$ pip install numpy
$ pip install velodyne_decoder

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


### Python demo
```bash
# Run patchwork++ with sequential point cloud inputs 
$ python examples/python/demo_sequential.py
```
