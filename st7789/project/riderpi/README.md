### 1. Board

#### 1.1 Board Info

Board Name: Raspberry Pi cm4.

SPI Pin: SCLK/MOSI/MISO/CS GPIO11/GPIO10/GPIO9/GPIO8.

GPIO Pin: RESET/CMD_DATA GPIO27/GPIO25.


### 2. Install

#### 2.1 Dependencies

Install the necessary dependencies.

```shell
sudo apt-get install libgpiod-dev pkg-config cmake -y
```

#### 2.2 Makefile

Build the project.

```shell
make
```

Install the project and this is optional.

```shell
sudo make install
```

Uninstall the project and this is optional.

```shell
sudo make uninstall
```


### 3.Usage
The Daemon creates a fifo /tmp/lcd0. There you can send raw data RGB565.

#### 3.1 Send from shell

For Example:
```
image:
ffmpeg -y -i input.jpg -s 320x240 -f rawvideo -pix_fmt rgb565 /tmp/lcd0

video:
ffmpeg -y -re -i input.avi -s 320x240 -f rawvideo -pix_fmt rgb565 /tmp/lcd0
```


#### 3.2 Send from ugly python
