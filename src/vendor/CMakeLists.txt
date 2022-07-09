# Vendor libraries 

# Doctest for some reason is not available for arm64 linux. Bundle it for now
# Consider switching to conan for C++ deps if more libraries end up here
add_subdirectory(doctest/v2.4.9)
