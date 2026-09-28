# MCB ESP32 WiFi Demo 1

This demo application consits of a MCB application and an ESP32 application.
The MCB application is the main application in the build and the ESP32 applciation
is included as a sub-applicaion n the main build as an external project.

In order to build the ESP32 application, the ESP-IDF needs to be setup, in
particular its environment variables (`IDF_PATH`, `PATH`).

The MCB application is pre-configured with the file `defaults.cmake`.
The ESP32 application is pre-configured with the file `esp32/sdkconfig.defaults`.
Building the top-level MCB main application will also build the ESP32 application.
Some additional convenience build targets are provided:

  * `esp32-build`
  * `esp32-flash`
  * `esp32-menuconfig`

