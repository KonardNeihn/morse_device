cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Die Anwendungs-Komponente liegt in "src" statt im Standardverzeichnis "main".
# Für natives ESP-IDF muss sie deshalb explizit registriert werden.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/src")

project(esp32-client)
