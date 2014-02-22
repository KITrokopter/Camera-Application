#include <opencv2/core/core.hpp>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>

#include <ros/ros.h>

#include "libfreenect.hpp"
#include "CvKinect.hpp"
#include "CvImageProcessor.hpp"
#include "Communicator.hpp"

bool startsWith(const char *prefix, const char *str)
{
	size_t lenPrefix = strlen(prefix),
	lenStr = strlen(str);
	return lenStr < lenPrefix ? false : strncmp(prefix, str, lenPrefix) == 0;
}

// Gets the ip address of the machine
char* getIpAddress()
{
	struct ifaddrs * ifAddrStruct = NULL;
	struct ifaddrs * ifa = NULL;
	void * tmpAddrPtr = NULL;
	char* result = 0;
	
	
	getifaddrs(&ifAddrStruct);
	
	for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
		// If address type is IPv4
		if (ifa ->ifa_addr->sa_family==AF_INET) {
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
			char addressBuffer[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
			
			if (startsWith("eth", ifa->ifa_name)) {
				result = (char*) malloc(INET_ADDRSTRLEN * sizeof(char));
				strcpy(result, addressBuffer);
				break;
			}
			
			// std::cout << ifa->ifa_name << " IP Address is " << addressBuffer << std::endl;
		} else
			// If address type is IPv6
			if (ifa->ifa_addr->sa_family==AF_INET6) {
            tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
			char addressBuffer[INET6_ADDRSTRLEN];
			inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
			// printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
		} 
	}
	
	if (ifAddrStruct!=NULL) {
		freeifaddrs(ifAddrStruct);
	}
	
	return result;
}

int main(int argc, char** argv)
{
	std::cout << "Starting Camera Application" << std::endl;
	
	char* ip = getIpAddress();
	
	if (ip == 0) {
		std::cout << "No ip address found. Terminating" << std::endl;
		exit(1);
	} else {
		// Replace . with _
		for (int i = 0; ip[i] != 0; i++) {
			if (ip[i] == '.') {
				ip[i] = '_';
			}
		}
	}
	
	// Create node name.
	char nodeName[80];
	strcpy(nodeName, "Camera_Application_");
	strcat(nodeName, ip);
	free(ip);
	
	ros::init(argc, argv, nodeName);
	
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	
	Freenect::Freenect freenect;
	CvKinect& device = freenect.createDevice<CvKinect>(0);
	CvImageProcessor* analyzer = new CvImageProcessor(&device, 0);
	
	// Do ROS stuff.
	Communicator comm(&device, analyzer);
	analyzer->setDataReceiver(&comm);
	analyzer->start();
	
	ROS_INFO("Initialized.");
	ros::spin();
	ROS_INFO("Shutting down.");
	
	analyzer->stop();
	
	ros::shutdown();
	std::cout << "ros::shutdown() sent" << std::endl;
	
	// Wait for ros to shutdown.
	while (ros::ok()) {
		usleep(10000);
	}
	
	std::cout << "Camera Application successfully terminated" << std::endl;
}
