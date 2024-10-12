### How to Contribute

We welcome contributions to the ZigUSB C6 project! Here’s how you can get involved:

1. **Development Environment**:
   - All development is done using Visual Studio Code (VSCode) with the ESP-IDF extension.
   - Ensure you have the ESP-IDF environment set up correctly. You can follow the [ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for detailed instructions.

2. **Cloning the Repository**:
   - Clone the repository to your local machine using:
     ```sh
     git clone https://github.com/xyzroe/ZigUSB_C6.git
     cd ZigUSB_C6
     ```

3. **Building the Firmware**:
   - The firmware is automatically built on GitHub after each commit using the `commit.sh` script.
   - To manually build the firmware locally, use the following commands:
     ```sh
     ./commit.sh
     ```

4. **Submitting Changes**:
   - Create a new branch for your feature or bug fix:
     ```sh
     git checkout -b feature-name
     ```
   - Make your changes and commit them with a descriptive message:
     ```sh
     git commit -m "Description of your changes"
     ```
   - Push your changes to your forked repository:
     ```sh
     git push origin feature-name
     ```
   - Open a pull request on GitHub, describing the changes you have made and why they should be merged.

5. **Code Reviews**:
   - All contributions will be reviewed by the maintainers. Please be responsive to feedback and make any necessary changes.

By following these steps, you can help improve the ZigUSB C6 project and contribute to its success. Thank you for your support!