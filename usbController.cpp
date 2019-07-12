#include "usbController.h"

/*int main()
{
    if(!initPort(file_descriptor))
        return 0;
    unsigned char buff_0000 = 0x11;
    write(file_descriptor, &buff_0000, 1);
    sleep(1);
    closePort();
    return 0;
}*/

bool usbRelayController::initPort(const char *dev) {
    //char dev[] ="/dev/ttyUSB1";
    file_descriptor = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    printf("00\n");
    if (-1 == file_descriptor)
        return false;
    printf("afd\n");

    struct termios options;
    int status = 1;
    tcgetattr(file_descriptor, &options);
    tcflush(file_descriptor, TCIOFLUSH);  //清空串口的缓冲区
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    status = tcsetattr(file_descriptor, TCSANOW, &options);
    if (status != 0)
        perror("tcsetattr fd1");
    printf("11\n");

    // set parity
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;   /* Clear parity enable */
    options.c_iflag &= ~INPCK;     /* Enable parity checking */
    options.c_cflag &= ~CSTOPB;

    /* Set input parity option */
    //if (parity != 'n')options.c_iflag |= INPCK ;
    options.c_iflag &= ~(INLCR | ICRNL);
    options.c_iflag &= ~(IXON);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag &= ~OPOST;   /*Output*/

    tcflush(file_descriptor, TCIFLUSH);
    options.c_cc[VTIME] = 100;//设置超时10秒
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(file_descriptor, TCSANOW, &options) != 0) {
        perror("SetupSerial 3");
        printf("22\n");
        return false;
    }
    return true;
}

void usbRelayController::closePort() {
    write(file_descriptor, &SIGNAL_CLOSE, 1);

}

void usbRelayController::openPort() {
    write(file_descriptor, &SIGNAL_OPEN, 1);
}




