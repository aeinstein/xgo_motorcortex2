#include "driver_st7789_basic.h"
//#include "shell.h"
#include <getopt.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <netinet/in.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pthread.h>

static int framesize = 320 * 240 * 2;
static char *framebuffer;
static char *fb_device = "/tmp/lcd0";
static int running = 1;

/**
 * @brief     signal handler
 * @param[in] signum signal number
 * @note      none
 */
static void a_sig_handler(int signum)
{
    if (SIGINT == signum)
    {
        st7789_interface_debug_print("st7789: close the server.\n");
        running = 0;
        exit(0);
    }
    
    return;
}

static void *read_framebuffer(){
    ssize_t bytes;
    struct stat sb;

    int rx_fd;
    uint8_t *rx;

    if (stat(fb_device, &sb) == -1) st7789_interface_debug_print("can't stat input file\n");

    rx_fd = open(fb_device, O_RDONLY);
    if (rx_fd < 0) st7789_interface_debug_print("can't create input device\n");

    rx = malloc(framesize);
    if (!rx) st7789_interface_debug_print("can't allocate rx buffer\n");

    while(running) {
        // read full frame from device

        int total_left = framesize;
        char *buffer_pointer = rx;

        while(total_left > 0) {
            bytes = read(rx_fd, buffer_pointer, total_left);

            if(bytes <= 0) {
                if(bytes < 0) {
                    st7789_interface_debug_print("device closed\n");
                    return NULL;
                }

                // no bytes so reset framepointer
                total_left = framesize;
                buffer_pointer = rx;
                usleep(10000);    // sleep when no data

            } else {
                total_left -= bytes;
                buffer_pointer += bytes;
            }
        }

        // buffer full write to framebuffer
        memcpy(framebuffer, rx, framesize);

        //st7789_interface_debug_print("picture complete\n");
        st7789_basic_draw_picture_16bits(0, 0, 319, 239, (uint16_t *)framebuffer);

        //  draw battery level
        //st7789_basic_string(250, 220, "Batt: 10%", 9, 0xFFFF, ST7789_FONT_16);
    }

    return NULL;
}

static void init_framebuffer(){
    struct stat sb;

    framebuffer = malloc(framesize);
    if(!framebuffer) st7789_interface_debug_print("cant allocate framebuffer");

    int i;
    for(i = 0; i < framesize; i++) {
        framebuffer[i] = 0;
    }

    // Wenn FIFO nicht existiert dann erstellen
    if (stat(fb_device, &sb) != 0) mkfifo(fb_device, 0777);

    st7789_basic_init();
    st7789_basic_display_on();

    pthread_t framereader;
    pthread_create (&framereader, NULL, read_framebuffer, "");
}


int main(void)
{
    daemon(0,0);
    uint8_t res;

    /* set the signal */
    signal(SIGINT, a_sig_handler);

    init_framebuffer();

    while(1) {
        sleep(1);
    }
}
