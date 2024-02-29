# Filters

## Usage with ament_cmake

Here is recommended approach on how to link `filters` to your project, using the `filter_base` target.

```cmake
find_package(filters CONFIG REQUIRED)

add_library(my_library)
# Other library stuff here

target_link_libraries(my_library PUBLIC filters::filter_base)
```

For more information, 
see the [ament_cmake](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html)
tutorial.
