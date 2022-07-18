# Laboratory of Applied Robotics project
**Team: [César González C.](https://github.com/Chesare9000)**

A final project for Laboratory of Applied Robotics class at **University of Trento, 2018-2019**. The main goal of the project is to implement computer vision and path planning algorithms for LEGO MINDSTORMS NXT robot. The code is written in C++.

## Dependencies

* [OpenCV 4.x.x](https://docs.opencv.org/4.6.0/d1/dfb/intro.html)
* [Tesseract 4.x.x](https://github.com/tesseract-ocr/tesseract)

### Ubuntu 22.04:

```bash
sudo apt install libopencv-dev libtesseract-dev
```

For installation instructions on other platforms refer to the official documentation.

## Building

```bash
git clone https://github.com/askhissak/lar-project.git
cd lar-project
make
```

## Usage

From the project root:

```bash
./build/apps/program data/dataset/data/img/2018-12-20-122754.jpg
```
