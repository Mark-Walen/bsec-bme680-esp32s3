# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v5.0/components/bootloader/subproject"
  "D:/git/esp32-s3/spi_tester/build/bootloader"
  "D:/git/esp32-s3/spi_tester/build/bootloader-prefix"
  "D:/git/esp32-s3/spi_tester/build/bootloader-prefix/tmp"
  "D:/git/esp32-s3/spi_tester/build/bootloader-prefix/src/bootloader-stamp"
  "D:/git/esp32-s3/spi_tester/build/bootloader-prefix/src"
  "D:/git/esp32-s3/spi_tester/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/git/esp32-s3/spi_tester/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/git/esp32-s3/spi_tester/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
