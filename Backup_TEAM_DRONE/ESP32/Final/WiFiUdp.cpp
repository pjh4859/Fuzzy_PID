/*
  Udp.cpp - UDP class for Raspberry Pi
  Copyright (c) 2016 Hristo Gochkov  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "WiFiUdp.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <string.h>
#include "Arduino.h"

#undef write
#undef read

WiFiUDP::WiFiUDP()
	: udp_server(-1)
	, server_port(0)
	, remote_port(0)
	, tx_buffer(0)
	, tx_buffer_len(0)
	, data_buffer(0)
	, data_buffer_len(0)
	, rx_buffer(0)
	, rx_len(1)
	, count(0)
{}

WiFiUDP::~WiFiUDP() {
	stop();
}

uint8_t WiFiUDP::begin(IPAddress address, uint16_t port) {
	stop();

	server_port = port;

	tx_buffer = new char[1460];
	if (!tx_buffer) {
		log_e("could not create tx buffer: %d", errno);
		return 0;
	}

	data_buffer = new char[1460];
	if (!data_buffer) {
		log_e("could not create data buffer: %d", errno);
		return 0;
	}

	if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		log_e("could not create socket: %d", errno);
		return 0;
	}

	int yes = 1;
	if (setsockopt(udp_server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
		log_e("could not set socket option: %d", errno);
		stop();
		return 0;
	}
	
	struct timeval timeout;
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	int optlen = sizeof(timeout);

	setsockopt(udp_server, SOL_SOCKET, SO_RCVTIMEO, (void*)&timeout, (socklen_t)optlen);

	//setsockopt(udp_server, SOL_SOCKET, SO_SNDTIMEO, (void*)&timeout, (socklen_t)optlen);
	
	struct sockaddr_in addr;
	memset((char*)&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(server_port);
	addr.sin_addr.s_addr = (in_addr_t)address;
	if (bind(udp_server, (struct sockaddr*) & addr, sizeof(addr)) == -1) {
		log_e("could not bind socket: %d", errno);
		stop();
		return 0;
	}
	fcntl(udp_server, F_SETFL, O_NONBLOCK);
	return 1;
}

uint8_t WiFiUDP::begin(uint16_t p) {
	return begin(IPAddress(INADDR_ANY), p);
}

uint8_t WiFiUDP::begin_rx(IPAddress address, uint16_t port) {
	stop();

	server_port = port;

	if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		log_e("could not create socket: %d", errno);
		return 0;
	}

	int yes = 1;
	if (setsockopt(udp_server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
		log_e("could not set socket option: %d", errno);
		stop();
		return 0;
	}

	struct timeval timeout;
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	int optlen = sizeof(timeout);

	setsockopt(udp_server, SOL_SOCKET, SO_RCVTIMEO, (void*)&timeout, (socklen_t)optlen);

	//setsockopt(udp_server, SOL_SOCKET, SO_SNDTIMEO, (void*)&timeout, (socklen_t)optlen);

	struct sockaddr_in addr;
	memset((char*)&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(server_port);
	addr.sin_addr.s_addr = (in_addr_t)address;
	if (bind(udp_server, (struct sockaddr*) & addr, sizeof(addr)) == -1) {
		log_e("could not bind socket: %d", errno);
		stop();
		return 0;
	}
	fcntl(udp_server, F_SETFL, O_NONBLOCK);
	return 1;
}

uint8_t WiFiUDP::begin_rx(uint16_t p) {
	return begin_rx(IPAddress(INADDR_ANY), p);
}


uint8_t WiFiUDP::beginMulticast(IPAddress a, uint16_t p) {
	if (begin(IPAddress(INADDR_ANY), p)) {
		if (a != 0) {
			struct ip_mreq mreq;
			mreq.imr_multiaddr.s_addr = (in_addr_t)a;
			mreq.imr_interface.s_addr = INADDR_ANY;
			if (setsockopt(udp_server, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
				log_e("could not join igmp: %d", errno);
				stop();
				return 0;
			}
			multicast_ip = a;
		}
		return 1;
	}
	return 0;
}

void WiFiUDP::stop() {
	if (tx_buffer) {
		delete[] tx_buffer;
		tx_buffer = NULL;
	}
	tx_buffer_len = 0;

	if (size_buffer) {
		delete[] size_buffer;
		size_buffer = NULL;
	}

	if (data_buffer) {
		delete[] data_buffer;
		data_buffer = NULL;
	}
	data_buffer_len = 0;

	if (rx_buffer) {
		cbuf* b = rx_buffer;
		rx_buffer = NULL;
		delete b;
	}
	if (udp_server == -1)
		return;
	if (multicast_ip != 0) {
		struct ip_mreq mreq;
		mreq.imr_multiaddr.s_addr = (in_addr_t)multicast_ip;
		mreq.imr_interface.s_addr = (in_addr_t)0;
		setsockopt(udp_server, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
		multicast_ip = IPAddress(INADDR_ANY);
	}
	close(udp_server);
	udp_server = -1;
}

int WiFiUDP::beginMulticastPacket() {
	if (!server_port || multicast_ip == IPAddress(INADDR_ANY))
		return 0;
	remote_ip = multicast_ip;
	remote_port = server_port;
	return beginPacket();
}

int WiFiUDP::beginPacket() {
	if (!remote_port)
		return 0;

	// allocate tx_buffer if is necessary
	if (!tx_buffer) {
		tx_buffer = new char[1460];
		if (!tx_buffer) {
			log_e("could not create tx buffer: %d", errno);
			return 0;
		}
	}

	tx_buffer_len = 0;

	// check whereas socket is already open
	if (udp_server != -1)
		return 1;

	if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		log_e("could not create socket: %d", errno);
		return 0;
	}

	fcntl(udp_server, F_SETFL, O_NONBLOCK);

	return 1;
}

int WiFiUDP::beginPacket(IPAddress ip, uint16_t port) {
	remote_ip = ip;
	remote_port = port;
	return beginPacket();
}

int WiFiUDP::beginPacket(const char* host, uint16_t port) {
	struct hostent* server;
	server = gethostbyname(host);
	if (server == NULL) {
		log_e("could not get host from dns: %d", errno);
		return 0;
	}
	return beginPacket(IPAddress((const uint8_t*)(server->h_addr_list[0])), port);
}

int WiFiUDP::endPacket() {
	struct sockaddr_in recipient;
	recipient.sin_addr.s_addr = (uint32_t)remote_ip;
	recipient.sin_family = AF_INET;
	recipient.sin_port = htons(remote_port);
	int sent = sendto(udp_server, tx_buffer, tx_buffer_len, 0, (struct sockaddr*) & recipient, sizeof(recipient));
	if (sent < 0) {
		log_e("could not send data: %d", errno);
		return 0;
	}
	return 1;
}

int WiFiUDP::beginSize() {
	if (!remote_port)
		return 0;

	// allocate tx_buffer if is necessary
	if (!size_buffer) {
		size_buffer = new size_t;
		if (!size_buffer) {
			log_e("could not create size buffer: %d", errno);
			return 0;
		}
	}

	// check whereas socket is already open
	if (udp_server != -1)
		return 1;

	if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		log_e("could not create socket: %d", errno);
		return 0;
	}

	fcntl(udp_server, F_SETFL, O_NONBLOCK);

	return 1;
}

int WiFiUDP::beginSize(IPAddress ip, uint16_t port) {
	remote_ip = ip;
	remote_port = port;
	return beginSize();
}

int WiFiUDP::beginSize(const char* host, uint16_t port) {
	struct hostent* server;
	server = gethostbyname(host);
	if (server == NULL) {
		log_e("could not get host from dns: %d", errno);
		return 0;
	}
	return beginSize(IPAddress((const uint8_t*)(server->h_addr_list[0])), port);
}


int WiFiUDP::endSize(size_t size) {
	struct sockaddr_in recipient;
	recipient.sin_addr.s_addr = (uint32_t)remote_ip;
	recipient.sin_family = AF_INET;
	recipient.sin_port = htons(remote_port);
	//int sent = sendto(udp_server,size_buffer, sizeof(size_buffer), 0, (struct sockaddr*) & recipient, sizeof(recipient));
	int sent = sendto(udp_server, (char*)&size, sizeof(size), 0, (struct sockaddr*) & recipient, sizeof(recipient));
	if (sent < 0) {
		log_e("could not send data: %d", errno);
		return 0;
	}
	return 1;
}


int WiFiUDP::endSize() {
	struct sockaddr_in recipient;
	recipient.sin_addr.s_addr = (uint32_t)remote_ip;
	recipient.sin_family = AF_INET;
	recipient.sin_port = htons(remote_port);
	//int sent = sendto(udp_server,size_buffer, sizeof(size_buffer), 0, (struct sockaddr*) & recipient, sizeof(recipient));
	int sent = sendto(udp_server, (char*)size_buffer, sizeof(size_t), 0, (struct sockaddr*) & recipient, sizeof(recipient));
	if (sent < 0) {
		log_e("could not send data: %d", errno);
		return -1;
	}
	return 1;
}

int WiFiUDP::beginData() {
	if (!remote_port)
		return 0;

	// allocate data_buffer if is necessary
	if (!data_buffer) {
		data_buffer = new char[1460];
		if (!data_buffer) {
			log_e("could not create tx buffer: %d", errno);
			return 0;
		}
	}

	//data_buffer_len = 0;
	// check whereas socket is already open
	if (udp_server != -1)
		return 1;

	if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		log_e("could not create socket: %d", errno);
		return 0;
	}

	fcntl(udp_server, F_SETFL, O_NONBLOCK);

	return 1;
}

int WiFiUDP::beginData(IPAddress ip, uint16_t port) {
	remote_ip = ip;
	remote_port = port;
	return beginData();
}

int WiFiUDP::beginData(const char* host, uint16_t port) {
	struct hostent* server;
	server = gethostbyname(host);
	if (server == NULL) {
		log_e("could not get host from dns: %d", errno);
		return 0;
	}
	return beginData(IPAddress((const uint8_t*)(server->h_addr_list[0])), port);
}


int WiFiUDP::endData() {
	struct sockaddr_in recipient;
	recipient.sin_addr.s_addr = (uint32_t)remote_ip;
	recipient.sin_family = AF_INET;
	recipient.sin_port = htons(remote_port);
	//int sent = sendto(udp_server,size_buffer, sizeof(size_buffer), 0, (struct sockaddr*) & recipient, sizeof(recipient));
	int sent = sendto(udp_server, data_buffer, data_buffer_len, 0, (struct sockaddr*) & recipient, sizeof(recipient));
	if (sent < 0) {
		log_e("could not send data: %d", errno);
		return -1;
	}
	return 1;
}



size_t WiFiUDP::size_write(size_t data) {
	size_buffer[0] = data;
	return 1;
}

size_t WiFiUDP::image_write(char data) {
	if (data_buffer_len == 1460) {
		//endData();
		data_buffer_len = 0;
	}
	data_buffer[data_buffer_len++] = data;
	return 1;
}

size_t WiFiUDP::image_write(const char* buffer, size_t size) {
	size_t i;
	for (i = 0; i < size; i++)
		image_write(buffer[i]);
	return i;
}

uint8_t WiFiUDP::begin_connect(IPAddress address, uint16_t port) {
	stop();

	server_port = port;

	tx_buffer = new char[1460];
	if (!tx_buffer) {
		log_e("could not create tx buffer: %d", errno);
		return 0;
	}

	data_buffer = new char[1460];
	if (!data_buffer) {
		log_e("could not create data buffer: %d", errno);
		return 0;
	}

	if ((udp_server = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		log_e("could not create socket: %d", errno);
		return 0;
	}

	int yes = 1;
	if (setsockopt(udp_server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
		log_e("could not set socket option: %d", errno);
		stop();
		return 0;
	}

	struct sockaddr_in addr;
	memset((char*)&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(server_port);
	addr.sin_addr.s_addr = (in_addr_t)address;
	if (bind(udp_server, (struct sockaddr*) & addr, sizeof(addr)) == -1) {
		log_e("could not bind socket: %d", errno);
		stop();
		return 0;
	}
	fcntl(udp_server, F_SETFL, O_NONBLOCK);

	connect(udp_server, (struct sockaddr*) & addr, sizeof(addr));
	return 1;
}

uint8_t WiFiUDP::begin_connect(uint16_t p) {
	return begin(IPAddress(INADDR_ANY), p);
}



size_t WiFiUDP::write(uint8_t data) {
	if (tx_buffer_len == 1460) {
		endPacket();
		tx_buffer_len = 0;
	}
	tx_buffer[tx_buffer_len++] = data;
	return 1;
}

size_t WiFiUDP::write(const uint8_t* buffer, size_t size) {
	size_t i;
	for (i = 0; i < size; i++)
		write(buffer[i]);
	return i;
}

int WiFiUDP::parsePacket() {
	if (rx_buffer)
		return 0;
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len;
	char* buf = new char[1460];
	if (!buf) {
		return 0;
	}
	if ((len = recvfrom(udp_server, buf, 1460, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		delete[] buf;
		if (errno == EWOULDBLOCK) {
			return 0;
		}
		log_e("could not receive data: %d", errno);
		return 0;
	}


	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	if (len > 0) {
		rx_buffer = new cbuf(len);
		rx_buffer->write(buf, len);
	}
	delete[] buf;
	return len;
}

int WiFiUDP::StartPacket() {
	if (rx_buffer)
		return 0;
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	char* buf = new char[20];
	if (!buf) {
		return 0;
	}
	recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen); //
	/*if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		delete[] buf;
		if (errno == EWOULDBLOCK) {
			return 0;
		}
		log_e("could not receive data: %d", errno);
		return 0;
	}*/

	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	if (strstr(buf, "START") != NULL)
	{
		char c = '1';
		beginPacket(remote_ip, remote_port);
		write(c);
		endPacket();
		len = 1;
	}
	delete[] buf;
	return len;
}

int WiFiUDP::receviePacket() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other);
	//char* buf = new char[20];
	char buf[20] = { 0, };
	rx_len = 1;
	recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen);
	//if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
	//	delete[] buf;
	//	if (errno == EWOULDBLOCK) {
	//		//return 0;
	//		return 1;
	//	}
	//	log_e("could not receive data: %d", errno);
	//	return 1;
	//}
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	//
	if (strstr(buf, "OK") != NULL)
	{
		rx_len = 2;
	}
	//if (strcmp(buf, "OK") == NULL)
	//{
	//	rx_len = 2;
	//}

	//delete[] buf;
	return rx_len;
}

int WiFiUDP::resendPacket() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other);
	char* buf = new char[20];
	rx_len = 2;
	//recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen);
	if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		delete[] buf;
		if (errno == EWOULDBLOCK) {
			//return 0;
			return 2;
		}
		log_e("could not receive data: %d", errno);
		return 2;
	}
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	if (strcmp(buf, "ReSend") == 0)
	{
		rx_len = 3;
	}
	delete[] buf;
	return rx_len;


}

int WiFiUDP::EndPacket() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other);
	char* buf = new char[20];
	rx_len = 2;
	//recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen);
	if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		delete[] buf;
		if (errno == EWOULDBLOCK) {
			//return 0;
			return 2;
		}
		log_e("could not receive data: %d", errno);
		return 2;
	}
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	if (strcmp(buf, "END") == 0)
	{
		rx_len = 0;
	}
	delete[] buf;
	return rx_len;


}


int WiFiUDP::SizePacket() {
	//if (rx_buffer)
	//	return 0;
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	char* buf = new char[20];
	if (!buf) {
		return 0;
	}
	//recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen); //
	if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		delete[] buf;
		if (errno == EWOULDBLOCK) {
			//return 0;
			return len;
		}
		log_e("could not receive data: %d", errno);
		return len;
	}
	char c = '1';
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	if (strstr(buf, "ReSendSize") != NULL)
	{
		beginPacket(remote_ip, remote_port);
		write(c);
		endPacket();
		len = 1;
	}
	delete[] buf;
	return len;
}


int WiFiUDP::StopPacket() {
	//if (rx_buffer)
	//	return 0;
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	//	char* buf = new char[20];
	char buf[20] = { 0, };
	if (!buf) {
		return 0;
	}

	//recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen); //
	if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		//delete[] buf;
		if (errno == EWOULDBLOCK) {
			//return 0;
			return len;
		}
		log_e("could not receive data: %d", errno);
		return len;
	}
	char c = '1';
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);

	if (strstr(buf, "STOPSIZE") != NULL)
	{
		beginPacket(remote_ip, remote_port);
		write(c);
		endPacket();
		len = 2;
	}
	//delete[] buf;
	return len;
}

int WiFiUDP::OKPacket() {
	if (rx_buffer)
		return 0;
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	//	char* buf = new char[20];
	char buf[20] = { 0, };
	if (!buf) {
		return 0;
	}

	//recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen); //
	if ((recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen)) == -1) {
		//delete[] buf;
		if (errno == EWOULDBLOCK) {
			//return 0;
			return len;
		}
		log_e("could not receive data: %d", errno);
		return len;
	}
	char c = '1';
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);

	if (strstr(buf, "OK") != NULL)
	{
		beginPacket(remote_ip, remote_port);
		write(c);
		endPacket();
		len = 3;
	}

	return len;
}


int WiFiUDP::StartP() {
	//START:
	struct sockaddr_in si_other;
	int slen = sizeof(si_other);
	char buf[20] = { 0, };
	char c = '1';
	//int temp;
	//int count = 0;
	do
	{
		recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);
		//count++;
	} while (!(strcmp(buf, "START") == 0));

	//if (!(temp == 0))
	//{
	//	sleep(10);
	//	goto START;
	//}

	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	beginPacket(remote_ip, remote_port);
	write(c);
	endPacket();

	return 1;

}

int WiFiUDP::recevieP() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other);
	//char* buf = new char[20];
	char buf[20] = { 0, };
	do
	{
		recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);
	} while (!(strcmp(buf, "OK") == 0));
	
	return 2;
}

int WiFiUDP::EndP() {
ENDp:
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	char buf[20] = { 0, };
	char c = '1';
	recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen); //
	if (strcmp(buf, "END") == 0)
		len = 1;
	//do
	//{
	//	recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);
	//} while (!(strcmp(buf, "END") == 0));
	return len;
}

int WiFiUDP::resendP() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other);
	char buf[20] = { 0, };
	rx_len = 2;
	recvfrom(udp_server, buf, 20, 0, (struct sockaddr*) & si_other, (socklen_t*)&slen);
	//recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);

	if (strcmp(buf, "ReSend") == 0)
	{
		rx_len = 3;
	}
	return rx_len;


}


int WiFiUDP::SizeP() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	char buf[20] = { 0, };
	char c = '1';
	dt = 0;
	pre_sec = millis();
	do
	{
		len++;
		sec = millis();
		dt = sec - pre_sec;
		recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);
		if (dt >= 5000)
			break;

	} while (!(strcmp(buf, "ReSendSize") == 0));

	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	beginPacket(remote_ip, remote_port);
	write(c);
	endPacket();
	return len;
}


int WiFiUDP::StopP() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 0;
	char buf[20] = { 0, };
	char c = '1';
	dt = 0;
	pre_sec = millis();
	do
	{
		sec = millis();
		dt = sec - pre_sec;
		recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);
		if (dt >= 5000)
			break;
	} while (!(strcmp(buf, "STOPSIZE") == 0));
	
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	beginPacket(remote_ip, remote_port);
	write(c);
	endPacket();
	return 2;
}

int WiFiUDP::OKP() {
	struct sockaddr_in si_other;
	int slen = sizeof(si_other), len = 1;
	char buf[20] = { 0, };
	char c = '1';
	dt = 0;
	pre_sec = millis();
	do
	{
		sec = millis();
		dt = sec - pre_sec;
		recvfrom(udp_server, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & si_other, (socklen_t*)&slen);
		if (dt >= 5000)
			break;
	} while (!(strcmp(buf, "OK") == 0));
	remote_ip = IPAddress(si_other.sin_addr.s_addr);
	remote_port = ntohs(si_other.sin_port);
	beginPacket(remote_ip, remote_port);
	write(c);
	endPacket();
	return 3;
}

int WiFiUDP::available() {
	if (!rx_buffer) return 0;
	return rx_buffer->available();
}

int WiFiUDP::read() {
	if (!rx_buffer) return -1;
	int out = rx_buffer->read();
	if (!rx_buffer->available()) {
		cbuf* b = rx_buffer;
		rx_buffer = 0;
		delete b;
	}
	return out;
}

int WiFiUDP::read(unsigned char* buffer, size_t len) {
	return read((char*)buffer, len);
}

int WiFiUDP::read(char* buffer, size_t len) {
	if (!rx_buffer) return 0;
	int out = rx_buffer->read(buffer, len);
	if (!rx_buffer->available()) {
		cbuf* b = rx_buffer;
		rx_buffer = 0;
		delete b;
	}
	return out;
}

int WiFiUDP::peek() {
	if (!rx_buffer) return -1;
	return rx_buffer->peek();
}

void WiFiUDP::flush() {
	if (!rx_buffer) return;
	cbuf* b = rx_buffer;
	rx_buffer = 0;
	delete b;
}

IPAddress WiFiUDP::remoteIP() {
	return remote_ip;
}

uint16_t WiFiUDP::remotePort() {
	return remote_port;
}

