//class header file
#include "car_interface_rpi/ros2can.hpp"

//TODO: might have to move these inclusions to the header
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <unistd.h>
#include <cstdio>
#include <cstring>

//namespace scoping
using namespace std;

void ros2can::send() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
	}

    strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
	}
    //TODO: might fail here due to pass by reference and pointer errors
    if (write(s, &data_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write");
	}

    if (close(s) < 0) {
		perror("Close"); 
	}

}

void ros2can::receive() {
    int s;
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
	}

    strcpy(ifr.ifr_name, "can0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
	}

    nbytes = read(s, &data_frame, sizeof(struct can_frame));

    if (nbytes < 0) {
		perror("Read");
	}

    if (close(s) < 0) {
		perror("Close");
	}
    
}
