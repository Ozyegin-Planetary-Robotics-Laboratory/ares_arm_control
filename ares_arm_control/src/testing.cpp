#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include "ak60.hpp"

// Function to continuously write data to a CSV file
void writeToCSV(const std::string& filename, std::mutex& mtx) {
    std::ofstream file(filename, std::ios::out | std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    
    while (true) {
        // write position bla bla data


    }
}

int main() {
    const std::string filename = "data.csv";

    // Create a mutex to synchronize access to the file
    std::mutex mtx;

    // Start a separate thread for writing to the CSV file
    std::thread writerThread(writeToCSV, filename, std::ref(mtx));
    
    TMotor::AK60Manager ak60(0x0B, "can0");

    // Main thread can do other tasks here while the writer thread writes to the CSV file

    // Join the writer thread to the main thread
    writerThread.join();

    return 0;
}
