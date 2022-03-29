# testcode
## Dependencies:
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PicoTree](https://github.com/Jaybro/pico_tree)

## Installing Dependencies:
- Installing Eigen:
    1. Clone the Eigen directory and follow the [install instructions](https://gitlab.com/libeigen/eigen/-/blob/master/INSTALL)
    2. You may have to do the following command so that it installs Eigen in /usr/include instead of /usr/local/include
 ```console 
$ mkdir build && cd build
$ cmake ../
$ sudo cmake . -DCMAKE_INSTALL_PREFIX=/usr/include
$ sudo make install

```

- Installing PicoTree:

    1. Clone the PicoTree directory somewhere accessible
    2. Run the following commands:

```console
$ cd src/pico_tree
$ sudo cp -r /pico_tree /usr/include
```
*You can now delete the PicoTree directory.*

# Building picoNN.cpp:
1. Navigate to the testcode directory
1. Run the following commands:

```console
$ mkdir build && cd build
$ cmake ../
$ cmake --build .
```

## OR
In the folder where cpp_fitting_testing.py is located run:
```console
$ bash build_cpp_fitting_testing.sh
```
