/*
 *  Udp.cpp: Library to send/receive UDP packets.
 *
 * NOTE: UDP is fast, but has some important limitations (thanks to Warren Gray for mentioning these)
 * 1) UDP does not guarantee the order in which assembled UDP packets are received. This
 * might not happen often in practice, but in larger network topologies, a UDP
 * packet can be received out of sequence.
 * 2) UDP does not guard against lost packets - so packets *can* disappear without the sender being
 * aware of it. Again, this may not be a concern in practice on small local networks.
 * For more information, see http://www.cafeaulait.org/course/week12/35.html
 *
 * MIT License:
 * Copyright (c) 2008 Bjoern Hartmann
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * bjoern@cs.stanford.edu 12/30/2008
 */

#ifndef _WIFIUDP_H_
#define _WIFIUDP_H_

#include <Arduino.h>
#include <Udp.h>
#include <cbuf.h>

class WiFiUDP : public UDP {
private:
  int udp_server;
  IPAddress multicast_ip;
  IPAddress remote_ip;
  uint16_t server_port;
  uint16_t remote_port;
  char * tx_buffer;
  size_t tx_buffer_len;
  cbuf * rx_buffer;

  char * data_buffer;
  size_t data_buffer_len;

  size_t* size_buffer;

  int rx_len;
  int count;

public:
  WiFiUDP();
  ~WiFiUDP();
  uint8_t begin(IPAddress a, uint16_t p);
  uint8_t begin(uint16_t p);
  uint8_t beginMulticast(IPAddress a, uint16_t p);
  void stop();
  int beginMulticastPacket();
  int beginPacket();
  int beginPacket(IPAddress ip, uint16_t port);
  int beginPacket(const char *host, uint16_t port);
  int endPacket();

  int beginSize();
  int beginSize(IPAddress ip, uint16_t port);
  int beginSize(const char* host, uint16_t port);
  int endSize();
  int endSize(size_t size);
  

  int beginData();
  int beginData(IPAddress ip, uint16_t port);
  int beginData(const char* host, uint16_t port);
  int endData();


  size_t size_write(size_t data);

  size_t image_write(char);
  size_t image_write(const char* buffer, size_t size);

  uint8_t begin_connect(IPAddress a, uint16_t p);
  uint8_t begin_connect(uint16_t p);

  size_t write(uint8_t);
  size_t write(const uint8_t *buffer, size_t size);
  int StartPacket();
  int parsePacket();
  int receviePacket();
  int resendPacket();
  int EndPacket();
  int SizePacket();
  int StopPacket();
  int OKPacket();

  int available();
  int read();
  int read(unsigned char* buffer, size_t len);
  int read(char* buffer, size_t len);
  int peek();
  void flush();
  IPAddress remoteIP();
  uint16_t remotePort();
};

#endif /* _WIFIUDP_H_ */
