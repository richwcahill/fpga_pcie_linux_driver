#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
	int length, l2, fd1, fd2, rc;
	char *nodename = "/dev/arria10_pcie_fpga";
	char message[] = "f\n";  //char message
	int message2 = 0xffaaeedd;  //int message
	int message3 = 0xdeadbeef;
	int message4 = 0xdaadcffc;
	int *ip;

	length = sizeof(message);
	//printf("CHAR message length: %d\n", length);
	//printf("INT message length: %d\n", l2);

	if (argc > 1)
		nodename = argv[1];

	ip = &message2;
	l2 = sizeof(message2);

	fd1 = open(nodename, O_RDWR);
	printf(" opened file descriptor first time  = %d\n", fd1);
	fd2 = open(nodename, O_RDWR);
	printf(" opened file descriptor second time  = %d\n\n", fd2);

	rc = write(fd1, ip, l2);
	printf("return code from write = %d on %d, message=%p\n", rc, fd1, ip);
	ip = &message3;
	l2 = sizeof(message3);
	rc = write(fd1, ip, l2);

	ip = &message4;
	l2 = sizeof(message4);
	rc = write(fd1, ip, l2);

	memset(message, 0, length);

	rc = read(fd1, message, l2);
	printf("\nreturn code from read = %d on %d, message=%s\n", rc, fd2, message);

	close(fd1);
	exit(EXIT_SUCCESS);
}