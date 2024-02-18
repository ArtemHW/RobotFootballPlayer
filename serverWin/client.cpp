#include <stdio.h>
#include <winsock2.h>
#include <iostream>
#include <string>

int main() {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        perror("WSAStartup failed");
        std::cout << "WSAStartup failed";
        return 1;
    }
    std::cout << "WSAStartup OK" << std::endl;

    // Create a socket
    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == INVALID_SOCKET) {
        perror("Socket creation failed");
        std::cout << "Socket creation failed";
        return 1;
    }
    std::cout << "Socket creation OK" << std::endl;

    // Define the server address
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    serverAddr.sin_addr.s_addr = inet_addr("192.168.137.1"); // Connect to localhost

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        perror("Connection to the server failed");
        std::cout << "Connection to the server failed" << std::endl;
        return 1;
    }
    std::cout << "Connected to the server" << std::endl;

    // Send an HTTP GET request
    std::string request = "GET / HTTP/1.1\r\nHost: 192.168.137.1:8080\r\n\r\n";
    int bytesSent = send(clientSocket, request.c_str(), request.size(), 0);
    if (bytesSent < 0) {
        std::cout << "Could not send the request";
    } else {
        std::cout << "Sent request to the server" << std::endl;
    }

    // Receive and display the server's response
    char buff[4096];
    int bytesRead;
    while ((bytesRead = recv(clientSocket, buff, sizeof(buff), 0)) > 0) {
        std::cout.write(buff, bytesRead);
    }

    closesocket(clientSocket);
    WSACleanup();

    return 0;
}
