#include <stdio.h>
#include <winsock2.h>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <chrono>
#include <atomic>
#include <unordered_map>
#include <fstream>

void CleanupThreads(std::vector<std::thread>& threads);
int sendResponse(SOCKET clientSocket, const std::string& response);
void HandleClient(SOCKET clientSocket);
void ReceiveData(SOCKET clientSocket, std::thread::id ThreadId);
void SendData(SOCKET clientSocket);
void writeStringToFile(const std::string& filename, const std::string& data);
std::string readStringFromFile(const std::string& filename);
void sleep_for(std::chrono::milliseconds duration);
void SendHtmlFile(SOCKET clientSocket, const std::string& filename);

std::vector<std::thread> threads;
// std::unordered_map<std::thread::id, std::atomic<bool>> threadFlags;
std::unordered_map<std::thread::id, uint8_t> threadFlags;

std::string avrBatVoltage;
std::string encoderRRpm;
std::string encoderLRpm;
std::string softPwmRValue;
std::string softPwmLValue;
std::string joyX;
std::string joyY;


int main() {

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "WSAStartup failed";
        return 1;
    }
    std::cout << "WSAStartup OK" << std::endl;

    // Create a socket
    SOCKET serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (serverSocket == INVALID_SOCKET) {
        std::cout << "Socket creation failed";
        return 1;
    }
    std::cout << "Socket creation OK" << std::endl;

    // Define the server address
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(8080);
    serverAddr.sin_addr.s_addr = inet_addr("192.168.137.1");

    // Bind the socket to the server address
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cout << "Binding failed";
        return 1;
    }
    std::cout << "Binding OK" << std::endl;

    // Listen for incoming connections
    if (listen(serverSocket, 10) == SOCKET_ERROR) {
        std::cout << "Listen failed";
        return 1;
    }
    std::cout << "Listen OK" << std::endl;

    // Start the cleanup thread in the background
    std::thread cleanupThread(CleanupThreads, std::ref(threads));

    while(true){
        //accept client request
        struct sockaddr_in clientAddr;
        int clientAddrLen = sizeof(clientAddr);
        SOCKET clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientAddrLen);
        if (clientSocket == INVALID_SOCKET) {
        std::cout << "Accepting connection failed" << std::endl;
        return 1;
        }
        std::cout << "Accepting connection OK" << std::endl;

        // Start a new thread to handle the client
        threads.emplace_back(HandleClient, clientSocket);

    }

    for (auto& thread : threads) {
        thread.join();
    }

    // Close sockets when done
    closesocket(serverSocket);
    WSACleanup();

    // Join the cleanup thread when the program is about to exit
    cleanupThread.join();

    return 0;
}

// Function to periodically clean up finished threads
void CleanupThreads(std::vector<std::thread>& threads) {
    while (true) {
        std::this_thread::sleep_for(std::chrono::minutes(1));  // Adjust the sleep duration as needed

        // Iterate over the vector and join finished threads
        auto it = threads.begin();
        while (it != threads.end()) {
            if ((threadFlags[it->get_id()] & (1<<1)) == (1<<1)) {
                it->join();
                it = threads.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void HandleClient(SOCKET clientSocket) {
    std::cout << "\033[36mHandleClient start\033[0m" << std::endl;

    // Create a new thread-specific flag for each client thread
    threadFlags[std::this_thread::get_id()] = 0;


    std::thread::id thisThreadId = std::this_thread::get_id();
    std::thread recvThread(ReceiveData, clientSocket, thisThreadId);

    // Sleep for x milliseconds (x*10^-3 seconds)
    std::cout << "\033[36mSleeping for x ms\033[0m" << std::endl;
    sleep_for(std::chrono::milliseconds(75));

    // Check the flag and start sendThread if needed
    std::cout << "\033[36mChecking startSendThread flag\033[0m" << std::endl;
    if ((threadFlags[thisThreadId] & 0x1) == 0x1) {
        std::cout << "\033[36mstartSendThread flag is true, starting sendThread\033[0m" << std::endl;
        std::thread sendThread(SendData, clientSocket);

        sendThread.join();
    } else {
        std::cout << "\033[36mstartSendThread flag is false, not starting sendThread\033[0m" << std::endl;
    }

    recvThread.join();

    std::cout << "\033[36mNumber of threads in vector threads " << threads.size() << "\033[0m" << std::endl;
    closesocket(clientSocket);
    std::cout << "\033[36mHandleClient end\033[0m" << std::endl;
    threadFlags[thisThreadId] |= (1<<1);
}

void ReceiveData(SOCKET clientSocket, std::thread::id ThreadId) {
    while(true) {
        char buff[30720] = { 0 };
        int bytes = recv(clientSocket, buff, sizeof(buff), 0);

        if (bytes <= 0) {
            if (bytes == 0) {
                // Connection closed by the client
                std::cout << "Connection closed by the client" << std::endl;
            } else {
                std::cout << "Error while receiving data or connection closed by the client" << std::endl;
            }
            // Close the socket and return from the function
            closesocket(clientSocket);
            return;
        } else {
            std::cout << "Received data: " << std::endl;
            // for (int i = 0; i < bytes; ++i) {
            //     std::cout << buff[i];
            // }
            std::cout << std::endl;
            // Parse the request and send a response as before.
            // ...
                // Check if it's a GET request
            if (strstr(buff, "GET /home") != NULL) {
                // Handle GET request
                std::string response = readStringFromFile("example.txt");
                // Calculate the content length based on the response
                std::string contentLengthHeader = "Content-Length: " + std::to_string(response.length()) + "\n";
                // Construct the complete response with dynamic Content-Length
                std::string httpResponse = "HTTP/1.1 200 OK\nContent-Type: text/html\n" + contentLengthHeader + "\n" + response;

                sendResponse(clientSocket, httpResponse);
            } else if(strstr(buff, "GET /app") != NULL) {
                SendHtmlFile(clientSocket, "index.html");
            } else if(strstr(buff, "GET /robot HTTP/1.1\r\nHost: 192.168.137.1") != NULL) {
                // Set the flag to start sendThread
                threadFlags[ThreadId] |= 0x1;

                std::string response = "robot_okay";
                sendResponse(clientSocket, response);
            } else if(strstr(buff, "GET /data_from_robot") != NULL){
                // Respond with the current values of the global variables
                std::string responseData = "{"
                    "\"avrBatVoltage\": " + avrBatVoltage + ","
                    "\"encoderRRpm\": " + encoderRRpm + ","
                    "\"encoderLRpm\": " + encoderLRpm + ","
                    "\"softPwmRValue\": " + softPwmRValue + ","
                    "\"softPwmLValue\": " + softPwmLValue +
                    "}";
                std::cout << "Response Data: " << responseData << std::endl;  // Debug output
                std::string contentLengthHeader = "Content-Length: " + std::to_string(responseData.length()) + "\n";
                std::string httpResponse = "HTTP/1.1 200 OK\nContent-Type: application/json\n" + contentLengthHeader + "\n" + responseData;
                sendResponse(clientSocket, httpResponse);
            } else if (strstr(buff, "POST /robot_data") != NULL) {
                // Find the start of the JSON content
                const char* jsonStart = strstr(buff, "{");
                if (jsonStart != NULL) {
                    // Find the end of the JSON content
                    const char* jsonEnd = strstr(buff, "}");
                    if (jsonEnd != NULL) {
                        // Calculate the length of the JSON content
                        size_t jsonLength = jsonEnd - jsonStart + 1;

                        // Extract the JSON content
                        std::string jsonContent(jsonStart, jsonLength);

                        // Search for variable names and extract values
                        size_t avrBatVoltagePos = jsonContent.find("\"avrBatVoltage\":");
                        if (avrBatVoltagePos != std::string::npos) {
                            size_t avrBatVoltageEnd = jsonContent.find(",", avrBatVoltagePos);
                            if (avrBatVoltageEnd != std::string::npos) {
                                std::string avrBatVoltageValue = jsonContent.substr(avrBatVoltagePos + 16, avrBatVoltageEnd - (avrBatVoltagePos + 16));
                                std::cout << "\033[32mavrBatVoltage: " << avrBatVoltageValue << "\033[0m" << std::endl;
                                avrBatVoltage = avrBatVoltageValue;
                            }
                        }

                        size_t encoderRRpmPos = jsonContent.find("\"EncoderR.rpm\":");
                        if (encoderRRpmPos != std::string::npos) {
                            size_t encoderRRpmEnd = jsonContent.find(",", encoderRRpmPos);
                            if (encoderRRpmEnd != std::string::npos) {
                                std::string encoderRRpmValue = jsonContent.substr(encoderRRpmPos + 15, encoderRRpmEnd - (encoderRRpmPos + 15));
                                std::cout << "\033[31mEncoderR.rpm: " << encoderRRpmValue << "\033[0m" << std::endl;
                                encoderRRpm = encoderRRpmValue;
                            }
                        }

                        size_t encoderLRpmPos = jsonContent.find("\"EncoderL.rpm\":");
                        if (encoderLRpmPos != std::string::npos) {
                            size_t encoderLRpmEnd = jsonContent.find(",", encoderLRpmPos);
                            if (encoderLRpmEnd != std::string::npos) {
                                std::string encoderLRpmValue = jsonContent.substr(encoderLRpmPos + 15, encoderLRpmEnd - (encoderLRpmPos + 15));
                                std::cout << "\033[33mEncoderL.rpm: " << encoderLRpmValue << "\033[0m" << std::endl;
                                encoderLRpm = encoderLRpmValue;
                            }
                        }

                        size_t softPwmRPwmValuePos = jsonContent.find("\"SoftPwmR.pwmValue\":");
                        if (softPwmRPwmValuePos != std::string::npos) {
                            size_t softPwmRPwmValueEnd = jsonContent.find(",", softPwmRPwmValuePos);
                            if (softPwmRPwmValueEnd != std::string::npos) {
                                std::string softPwmRPwmValue = jsonContent.substr(softPwmRPwmValuePos + 20, softPwmRPwmValueEnd - (softPwmRPwmValuePos + 20));
                                std::cout << "\033[31mSoftPwmR.pwmValue: " << softPwmRPwmValue << "\033[0m" << std::endl;
                                softPwmRValue = softPwmRPwmValue;
                            }
                        }

                        size_t softPwmLPwmValuePos = jsonContent.find("\"SoftPwmL.pwmValue\":");
                        if (softPwmLPwmValuePos != std::string::npos) {
                            size_t softPwmLPwmValueEnd = jsonContent.find(",", softPwmLPwmValuePos);
                            if (softPwmLPwmValueEnd != std::string::npos) {
                                std::string softPwmLPwmValue = jsonContent.substr(softPwmLPwmValuePos + 20, softPwmLPwmValueEnd - (softPwmLPwmValuePos + 20));
                                std::cout << "\033[33mSoftPwmL.pwmValue: " << softPwmLPwmValue << "\033[0m" << std::endl;
                                softPwmLValue = softPwmLPwmValue;
                            }
                        }

                        size_t joyXValuePos = jsonContent.find("\"joyX\":");
                        if (joyXValuePos != std::string::npos) {
                            size_t joyXValueEnd = jsonContent.find(",", joyXValuePos);
                            if (joyXValueEnd != std::string::npos) {
                                std::string joyXValue = jsonContent.substr(joyXValuePos + 7, joyXValueEnd - (joyXValuePos + 7));
                                std::cout << "\033[33mjoyX: " << joyXValue << "\033[0m" << std::endl;
                            }
                        }

                        size_t joyYValuePos = jsonContent.find("\"joyY\":");
                        if (joyYValuePos != std::string::npos) {
                            size_t joyYValueEnd = jsonContent.find("}", joyYValuePos);
                            if (joyYValueEnd != std::string::npos) {
                                std::string joyYValue = jsonContent.substr(joyYValuePos + 7, joyYValueEnd - (joyYValuePos + 7));
                                std::cout << "\033[33mjoyY: " << joyYValue << "\033[0m" << std::endl;
                            }
                        }
                    }
                }
            } else if(strstr(buff, "POST /joystick_data") != NULL) {
                // Find the start of the JSON content
                const char* jsonStart = strstr(buff, "{");
                if (jsonStart != NULL) {
                    // Find the end of the JSON content
                    const char* jsonEnd = strstr(buff, "}");
                    if (jsonEnd != NULL) {
                        // Calculate the length of the JSON content
                        size_t jsonLength = jsonEnd - jsonStart + 1;

                        // Extract the JSON content
                        std::string jsonContent(jsonStart, jsonLength);

                        // Search for variable names and extract values
                        size_t joyXPos = jsonContent.find("\"joy3X\":");
                        if (joyXPos != std::string::npos) {
                            size_t joyXEnd = jsonContent.find(",", joyXPos);
                            if (joyXEnd != std::string::npos) {
                                std::string joyXValue = jsonContent.substr(joyXPos + 9, joyXEnd-1 - (joyXPos + 9));
                                joyX = joyXValue;
                                std::cout << "\033[32mjoyX: " << joyX << "\033[0m" << std::endl;
                            }
                        }

                        size_t joyYPos = jsonContent.find("\"joy3Y\":");
                        if (joyYPos != std::string::npos) {
                            size_t joyYEnd = jsonContent.find("}", joyYPos);
                            if (joyYEnd != std::string::npos) {
                                std::string joyYValue = jsonContent.substr(joyYPos + 9, joyYEnd-1 - (joyYPos + 9));
                                joyY = joyYValue;
                                std::cout << "\033[32mjoyY: " << joyY << "\033[0m" << std::endl;
                            }
                        }

                        std::string response = "HTTP/1.1 200 OK\nContent-Type: text/plain\nContent-Length: 2\n\nOK";
                        sendResponse(clientSocket, response);
                    }
                }
            } else {
                // Handle other request methods
                std::string response = "HTTP/1.1 501 Not Implemented\nContent-Type: text/html\nContent-Length: 28\n\n";
                response += "<html><h1>Not Implemented</h1></html>";
                sendResponse(clientSocket, response);
            }
        }
        if((threadFlags[ThreadId] & 0x1) != 0x1) {
            break;
        }
    }
}

void SendData(SOCKET clientSocket) {
    while (true) {

        std::string message = "JOY_X" + joyX + "Y" + joyY + "_";
        if(sendResponse(clientSocket, message) != 0) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(81));
    }
}

int sendResponse(SOCKET clientSocket, const std::string& response) {
    int bytesSent = 0;
    int totalBytesSent = 0;
    while (totalBytesSent < response.size()) {
        bytesSent = send(clientSocket, response.c_str() + totalBytesSent, response.size() - totalBytesSent, 0);
        if (bytesSent <= 0) {
            std::cout << "Could not send response" << std::endl;
            return 1;
        }
        totalBytesSent += bytesSent;
    }
    std::cout << "Sent response to client" << std::endl;
    return 0;
}

// Function to write a string to a file
void writeStringToFile(const std::string& filename, const std::string& data) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << data;
        file.close();
        std::cout << "String written to file: " << filename << std::endl;
    } else {
        std::cerr << "Error: Unable to open file for writing." << std::endl;
    }
}

// Function to read a string from a file
std::string readStringFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string content;
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            content += line;
            content += '\n'; // Add newline to separate lines
        }
        file.close();
        std::cout << "String read from file: " << filename << std::endl;
    } else {
        std::cerr << "Error: Unable to open file for reading." << std::endl;
    }
    return content;
}

// Function to sleep for a specified duration using steady_clock
void sleep_for(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
        std::this_thread::yield();  // Allow other threads to run
    }
}

// Implementation of SendHtmlFile function
void SendHtmlFile(SOCKET clientSocket, const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);

    if (!file.is_open()) {
        std::string response = "HTTP/1.1 404 Not Found\nContent-Type: text/html\nContent-Length: 21\n\n<html><h1>404 Not Found</h1></html>";
        sendResponse(clientSocket, response);
        return;
    }

    // Read the content of the HTML file
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // Calculate the content length based on the file size
    std::string contentLengthHeader = "Content-Length: " + std::to_string(content.size()) + "\n";

    // Construct the complete response with dynamic Content-Length
    std::string httpResponse = "HTTP/1.1 200 OK\nContent-Type: text/html\n" + contentLengthHeader + "\n" + content;

    // Send the response to the client
    sendResponse(clientSocket, httpResponse);
}