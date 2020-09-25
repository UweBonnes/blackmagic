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
#include "NetworkInterface.h"

#include "general.h"
#include "gdb_if.h"

#define DEFAULT_PORT 2000

NetworkInterface* network;
static int16_t port = DEFAULT_PORT;
static TCPSocket *sockServer;
static TCPSocket *sockClient = nullptr;


static void startServer()
{
    nsapi_error_t error = NSAPI_ERROR_OK;
    // Connect to the network with the default networking interface

    // using non blocking, async event driven
    network = NetworkInterface::get_default_instance();
    if (!network) {
        printf("Cannot connect to the network, see serial output\n");
    } 

    network->set_blocking(true);
    network->connect();

    sockServer = new TCPSocket;
    
    sockServer->open(network);
    if (error != NSAPI_ERROR_OK) {
        printf("sockServer open error: %i\n", error);
    }

    sockServer->set_blocking(true);
    int optval = 1;
    sockServer->setsockopt(NSAPI_SOCKET, NSAPI_REUSEADDR,  &optval, sizeof(optval));
    //sockServer->sigio(callback(serverStateChanged));
    
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

void waitForConnection()
{
	nsapi_error_t error = NSAPI_ERROR_OK;

	printf("wait for connection...\n");
    sockClient = sockServer->accept(&error);
    if (sockClient && (error == NSAPI_ERROR_OK)) {
        SocketAddress sockAddrClient;
        sockClient->getpeername(&sockAddrClient);
        printf("connected to %s\n", sockAddrClient.get_ip_address());
    }
}

void closeSocket()
{
	if (sockClient) {
		sockClient->close();
		sockClient = nullptr;
	}
}

#if 0
static void stopServer()
{
    if (sockServer) {
        sockServer->close();
        delete sockServer;
        sockServer = nullptr;
        printf("server stopped\n");
    }
}
#endif

int gdb_if_init(void)
{
	startServer();

	printf("Listening on TCP: %4d\n", port);

	return 0;
}

unsigned char gdb_if_getchar(void)
{
	unsigned char ret;

	while (sockClient == nullptr) {
		waitForConnection();
	}

	sockClient->set_blocking(true);
	nsapi_size_or_error_t sizeOrError = NSAPI_ERROR_OK;
	while(sizeOrError == NSAPI_ERROR_OK) { 	//  || sizeOrError == NSAPI_ERROR_NO_CONNECTION) {
		sizeOrError = sockClient->recv(&ret, 1);
	}

	if (sizeOrError == 1) {
		return ret;
	} else
	{
		printf("gdb_if_getchar(): %d\n", sizeOrError);
		if (sizeOrError < 0) {
			if (sockClient) {
				sockClient->close();
				sockClient = nullptr;
			}
		}
		return '+';
	}
}

unsigned char gdb_if_getchar_to(int timeout)
{
	unsigned char ret='+';

	while (sockClient == nullptr) {
		waitForConnection();
	}

	sockClient->set_timeout(timeout);
	nsapi_size_or_error_t sizeOrError = NSAPI_ERROR_OK;
	while(sizeOrError == NSAPI_ERROR_OK) { 		// || sizeOrError == NSAPI_ERROR_NO_CONNECTION) {
		sizeOrError = sockClient->recv(&ret, 1);
	}

	if (sizeOrError == 1) {
		return ret;
	} else
	{
		if (sizeOrError != NSAPI_ERROR_WOULD_BLOCK) {
			printf("gdb_if_getchar_to(): %d  to:%d\n", sizeOrError, timeout);
			if (sockClient) {
				sockClient->close();
				sockClient = nullptr;
			}
			ThisThread::sleep_for(5ms);
		}
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
			nsapi_size_or_error_t sizeOrError = NSAPI_ERROR_OK;
			sizeOrError = sockClient->send(sendbuf, bufsize);
			sendbuf[bufsize] = 0;
			//printf("S %d:'%s'\n", bufsize, (const char*)sendbuf);
			bufsize = 0;

			if (sizeOrError < 0) {
				sockClient->close();
				sockClient = nullptr;
			}
		}
	}
}
