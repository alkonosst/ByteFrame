cmake_minimum_required(VERSION 3.14)
project(ByteFrame VERSION 0.1.0 LANGUAGES CXX)

# Create an INTERFACE library.
add_library(ByteFrame INTERFACE)

# Alias with author prefix. Consumers link "alkonosst::ByteFrame".
add_library(alkonosst::ByteFrame ALIAS ByteFrame)

# Specify the include directories for the library.
target_include_directories(ByteFrame INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Set a C++ standard requirement for the library. This will also propagate to consumers of the library.
target_compile_features(ByteFrame INTERFACE cxx_std_17)