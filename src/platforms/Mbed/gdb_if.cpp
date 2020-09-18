/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements a transparent channel over which the GDB Remote
 * Serial Debugging protocol is implemented.  This implementation for Linux
 * uses a TCP server on port 2000.
 */

#include "mbed.h"
#include "myEthernet.h"

#include "general.h"
#include "gdb_if.h"

#define DEFAULT_PORT 2000

static int16_t port = DEFAULT_PORT;
static TCPSocket *sockServer;
static TCPSocket *sockClient = nullptr;

static void serverStateChanged() 
{
    printf("sockServer state changed\n");
    if (sockServer == nullptr) {
        return;
    }
    
    nsapi_error_t error = NSAPI_ERROR_OK;
    sockClient = sockServer->accept(&error);
    if (sockClient && (error == NSAPI_ERROR_OK)) {
        SocketAddress sockAddrClient;
        sockClient->getpeername(&sockAddrClient);
        printf("connected to %s\n", sockAddrClient.get_ip_address());

        // string msg = "Hello from Mbed\n";
        // sockClient->send(msg.c_str(), msg.length());

        // sockClient->close();
        // printf("client connection closed\n");
    }
}

static void startServer()
{
    nsapi_error_t error = NSAPI_ERROR_OK;
    sockServer = new TCPSocket;
    
    sockServer->open(network);
    if (error != NSAPI_ERROR_OK) {
        printf("sockServer open error: %i\n", error);
    }

    sockServer->set_blocking(false);
    int optval = 1;
    sockServer->setsockopt(NSAPI_SOCKET, NSAPI_REUSEADDR,  &optval, sizeof(optval));
    sockServer->sigio(callback(serverStateChanged));
    
    error = sockServer->bind(port);
    if (error != NSAPI_ERROR_OK) {
        printf("sockServer bind error: %i\n", error);
    }

    error = sockServer->listen(1);
    if (error != NSAPI_ERROR_OK) {
        printf("sockServer listen error: %i\n", error);
    }
    
    printf("server started\n");
}

static void stopServer()
{
    if (sockServer) {
        sockServer->close();
        delete sockServer;
        sockServer = nullptr;
        printf("server stopped\n");
    }
}


extern "C" {

int gdb_if_init(void)
{
	cbStartServer = startServer;
	cbStopServer = stopServer;

	printf("Listening on TCP: %4d\n", port);

	return 0;
}

unsigned char gdb_if_getchar(void)
{
	unsigned char ret;

	while (sockClient == nullptr) {
		ThisThread::sleep_for(10ms);
	}

	sockClient->set_blocking(true);
	if (sockClient->recv(&ret, 1) > 0) {
		return ret;
	} else
	{
		return '+';
	}
}

unsigned char gdb_if_getchar_to(int timeout)
{
	unsigned char ret='+';

	while (sockClient == nullptr) {
		ThisThread::sleep_for(10ms);
	}

	sockClient->set_timeout(timeout);
	if (sockClient->recv(&ret, 1) > 0) {
		return ret;
	} else
	{
		return -1;
	}
}

void gdb_if_putchar(unsigned char c, int flush)
{
	static int bufsize = 0;
	static uint8_t sendbuf[2048];
	
	while (sockClient == nullptr) {
		ThisThread::sleep_for(10ms);
	}

	if (sockClient) {
		sockClient->set_blocking(true);
		sendbuf[bufsize++] = c;
		if (flush || (bufsize == sizeof(sendbuf))) {
			sockClient->send(sendbuf, bufsize);
			sendbuf[bufsize] = 0;
			//printf("S %d:'%s'\n", bufsize, (const char*)sendbuf);
			bufsize = 0;
		}
	}
}

} // extern "C"
