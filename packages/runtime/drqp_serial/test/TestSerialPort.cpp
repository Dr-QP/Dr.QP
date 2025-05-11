#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <pty.h>
#include <errno.h>
#include <string.h>

int main() {
    int master_fd = -1, slave_fd = -1;
    char slave_name[256] = {};
    pid_t pid = 0;

    // Open a pty
    if (openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) == -1) {
        std::cerr << "Error opening pty: " << strerror(errno) << std::endl;
        return 1;
    }

    std::cout << "Slave name: " << slave_name << std::endl;

    // Fork the process
    pid = fork();
    if (pid == -1) {
        std::cerr << "Error forking process: " << strerror(errno) << std::endl;
        return 1;
    }

    if (pid == 0) {
        // Child process: execute a command in the pty
        close(master_fd); // Close the master end in the child
        setsid(); // Create a new session
        dup2(slave_fd, STDIN_FILENO);  // Duplicate slave to stdin
        dup2(slave_fd, STDOUT_FILENO); // Duplicate slave to stdout
        dup2(slave_fd, STDERR_FILENO); // Duplicate slave to stderr
        close(slave_fd); // Close the original slave fd

        // Execute a command (e.g., bash)
        execlp("bash", "bash", nullptr);
        std::cerr << "Error executing command: " << strerror(errno) << std::endl;
        return 1;
    } else {
        // Parent process: communicate with the child through the pty
        close(slave_fd); // Close the slave end in the parent
        std::string command;
        char buffer[1024];
        ssize_t bytes_read;

        while (true) {
            std::cout << "Enter command: ";
            std::getline(std::cin, command);

            // Send the command to the child process
            write(master_fd, command.c_str(), command.length());
            write(master_fd, "\n", 1); // Add newline character

            // Read the output from the child process
            while ((bytes_read = read(master_fd, buffer, sizeof(buffer) - 1)) > 0) {
                buffer[bytes_read] = '\0';
                std::cout << buffer;
            }
            if (bytes_read < 0) {
              std::cerr << "Error reading from pty: " << strerror(errno) << std::endl;
              break;
            }
            if (std::cin.eof()){
                break;
            }
        }

        // Wait for the child process to finish
        wait(nullptr);
        close(master_fd);
    }

    return 0;
}
