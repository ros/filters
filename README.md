: filters is now a separate stack. In previous releases, it was part of
[common](common).

## Pre-requisite: Pluginlib

Filters are closely linked to
[pluginlib](https://index.ros.org/p/pluginlib). Please make sure that
you are familiar with the
[pluginlib documentation](https://index.ros.org/p/pluginlib) before
continuing.

## Using Filters

It is recommended to use a `filters::FilterChain` whenever using more
than one instances of `filters::Filter`. However, you need to
understand the base Filter C++ API first before using the Filter Chain
API.

### Filter

The core of the `filters` Package is a templated base class
`filters::FilterBase<TemplateType>` which defines the external API
to any filter. The methods are `configure()` and `update()`. It also
provides helper methods for a Filter implementation. To use a Filter
simply instantiate it in your code.

Filters are configured from parameters on the
[Parameter Server](Parameter Server). The configure method passes in the
parameter namespace to read from of the Parameter Server.

#### Currently Implemented Filters

- [MeanFilter](#meanfilter)
- [MedianFilter](#medianfilter)
- [IncrementFilter](#incrementfilter)
- [TransferFunctionFilter](#transferfunctionfilter)
  <a id="FilterParameters"></a>

#### Filter ROS Parameters

A Filter expects to find a map with three elements: `name`, `type`
and `params`.

`name` (`string`) a unique name for debugging purposes within the
chain being used. `type` (`string`) the Filter's typename as
declared in it's [pluginlib](https://index.ros.org/p/pluginlib)
registration. `params` (`map`) data to present to the Filter
implementation.

#### Example configuration



```yaml
parameter_namespace_passed_to_configure:
  name: unique_name
  type: FilterName
  params: { param1: a, param2: b}
```



### Filter Chain

The `filters::FilterChain` class has been designed to facilitate
dynamically loading a sequence of Filters based on runtime parameters.
The Filter Chain is configured from the
[Parameter Server](Parameter Server) just like Filters. Based on the
parameters the Filter Chain will dynamically load the Filters by their
name.

The public API looks just like the Filter API with `configure()` and
`update()` methods.

#### Filter Chain ROS Parameters

`filter_chain` (list(Filters)) Parameter containing properly formatted
Filters. See
[Filter ROS Parameters](#FilterParameters).

#### Example configuration



```
param_namespace_passed_to_configure:
  filter_chain:
    -
      name: median_test_unique
      type: MultiChannelMedianFilterDouble
      params: {number_of_observations: 5}
    - 
      name: median_test2
      type: MultiChannelMedianFilterDouble
      params: {number_of_observations: 5}
```



#### Using a Filter Chain

Here's an example of a multi-channel filter chain for doubles. To do a
non-vector style use a `filters::FilterChain<double>` and change
`in` and `out` to type `double`.

Initialization: 

```
filters::MultiChannelFilterChain<double> chain("double"); //Node template type and argument must match
chain.configure("param_namespace_passed_to_configure");
std::vector<double> in, out;
```



Process data: assuming in is populated 

```
chain.update(in, out);
```



## Implementing a Filter

Implementing a `Filter` is simply a specific form of a dynamically
loaded class for [pluginlib](https://index.ros.org/p/pluginlib).

### Steps to create a filter

1\. Create a class which inherits from `FilterBase<T>` a. Implement
`configure()` method a. Implement `update()` method 1. In the source
for the class call 

```
PLUGINLIB_REGISTER_PLUGIN(UniqueFullClassNameSoSpecialCharacters, my_package::ClassName, filters::FilterBase<Type>)
```

: A new simpler plugin macro
is now recommended: 

```
PLUGINLIB_EXPORT_PLUGIN(my_package::ClassName, filters::FilterBase<Type>)
```

 1. Create a plugin
description file, see [pluginlib](https://index.ros.org/p/pluginlib) 1.
Add an export to your [manifest.xml](Manifest) pointing to the plugin
description file, see [pluginlib](https://index.ros.org/p/pluginlib) 1.
Make sure you depend on the `filters` package in your
[manifest.xml](Manifest).

## MultiChannelFilterBase and MultiChannelFilterChain

This whole document refers simply to `filters::FilterBase` and
`filters::FilterChain`. However there are `MultiChannel` versions of
both. Instead of the templated data type, `T`, they expect
`std::vector<T>` on the update calls. And the configure method has
one extra argument which is the number of channels expected(aka how many
elements are expected in the `vector`).

## MeanFilter

The
[MeanFilter](http://www.ros.org/doc/api/filters/html/classfilters_1_1MeanFilter.html)
calculates the output for `N` one-dimensional digital filters. Where
the input, `x`, is a (`N` x 1) vector of inputs and the output,
`y`, is a (`N` x 1) vector of outputs. The filter is described by
number of observations, `obs`, and implemented using the standard
difference equation:

y\[n\] = (x\[n\] + x\[n-1\] + \... + x\[n-obs\])/obs

#### ROS API

##### Parameters

- `~name` (`string`, default: `Required`): Name of the filter
- `~type`
- (`string`, default: `Required`): The filter's typename as declared in
  it's [pluginlib](https://index.ros.org/p/pluginlib) registration:
  `MultiChannelMeanFilterDouble` or `MeanFilterDouble`
- `~params/number_of_observations` (`int`, default: `Required`): The
  order of the filter

**Example Configuration:** 

```
FifthOrder:
  name: my_filter
  type: MultiChannelMeanFilterDouble
  params:
    number_of_observations: 5
```

## MedianFilter

The
[MedianFilter](http://www.ros.org/doc/api/filters/html/classfilters_1_1MedianFilter.html)
calculates the output for N one-dimensional digital filters. Where the
input, x, is a (N x 1) vector of inputs and the output, y, is a (N x 1)
vector of outputs. The filter is described by number of observations,
obs.

y\[n\] = median(x\[n\] + x\[n-1\] + \... + x\[n-obs\])

#### ROS API

##### Parameters

- `~name` (`string`, default: `Required`): Name of the filter
- `~type`
- (`string`, default: `Required`): The filter's typename as declared in
  it's [pluginlib](https://index.ros.org/p/pluginlib) registration:
  `MultiChannelMedianFilterDouble` or `MedianFilterDouble`
- `~params/number_of_observations` (`int`, default: `Required`): The
  order of the filter

**Example Configuration:** 

```
FifthOrder:
  name: my_filter
  type: MultiChannelMedianFilterDouble
  params:
    number_of_observations: 5
```

## IncrementFilter

The
[IncrementFilter](http://www.ros.org/doc/api/filters/html/classfilters_1_1IncrementFilter.html)
calculates the output for `N` one-dimensional digital filters. Where
the input, `x`, is a (`N` x 1) vector of inputs and the output,
`y`, is a (`N` x 1) vector of outputs. The filter is implemented
using the standard difference equation:

y\[n\] = x\[n\] + 1

#### ROS API

##### Parameters

- `~name` (`string`, default: `Required`): Name of the filter
- `~type`
- (`string`, default: `Required`): The filter's typename as declared in
  it's [pluginlib](https://index.ros.org/p/pluginlib) registration:
  `MultiChannelIncrementFilterDouble` or `IncrementFilterDouble`

**Example Configuration:** 

```
Increment:
  name: my_filter
  type: MultiChannelIncrementFilterDouble
```

## TransferFunctionFilter

The
[TransferFunctionFilter](http://www.ros.org/doc/api/filters/html/classfilters_1_1TransferFunctionFilter.html)
calculates the output for N one-dimensional digital filters. Where the
input, x, is a (N x 1) vector of inputs and the output, y, is a (N x 1)
vector of outputs. The filter is described by vectors a and b and
implemented using the standard difference equation:

a\[0\]\*y\[n\] = b\[0\]\*x\[n\] + b\[1\]\*x\[n-1\] + \... +
b\[n_{b}\]\*x\[n-n_{b}\] - a\[1\]\*y\[n-1\] - \... -
a\[n_{a}\]\*y\[n-n_{a}\]

If a\[0\] is not equal to 1, the coefficients are normalised by a\[0\].

#### ROS API

##### Parameters

- `~name` (`string`, default: `Required`): Name of the filter
- `~type`
- (`string`, default: `Required`): The filter's typename as declared in
  it's [pluginlib](https://index.ros.org/p/pluginlib) registration:
  `MultiChannelTransferFunctionFilterDouble` or
  `TransferFunctionFilterDouble`
- `~params/a` (`double`, default: `Required`): A vector of coefficients
- `~params/b` (`double`, default: `Required`): A vector of coefficients

**Example Configuration:** 

```
LowPass:
  name: my_filter
  type: MultiChannelTransferFunctionFilterDouble
  params:
    a: [1.0, -0.509525449494429]
    b: [0.245237275252786, 0.245237275252786]
```

## Tutorials

- [Implementing a simple filter](#implementing-a-simple-filter):
  This will describe how to implement a simple filter. This filter will
  simply add 1 to any incoming value. This filter exists in the filter
  package. The version for this tutorial has been isolated from other
  example files in the filters package.

## Troubleshooting

### Dynamic Loading Errors

Filters are a thin wrapper around pluginlib. If you're having trouble
with loading filters see
[Troubleshooting](https://index.ros.org/p/pluginlib)

### Reporting Errors

If you encounter a problem that isn't addressed on this page, please
[file a ticket](https://code.ros.org/trac/ros-pkg/newticket?component=common&keywords=filtersf)

The current list of open issues is available on
[trac](https://code.ros.org/trac/ros-pkg/query?status=assigned&status=new&status=reopened&group=milestone&order=priority&col=id&col=summary&col=status&col=type&col=priority&col=milestone&col=component&component=common&keywords=~filters)

## Implementing a simple filter

- **Description:** This will describe how to implement a simple filter.
  This filter will simply add 1 to any incoming value. This filter
  exists in the filter package. The version for this tutorial has been
  isolated from other example files in the filters package.
- **Keywords:** filters
- **Tutorial Level:** BEGINNER
- **Next Tutorial:** How to use filters with respect to laser data
  [Laser filtering using the filter nodes](https://index.ros.org/p/laser_filters)
  [Laser filtering in C++](https://index.ros.org/p/laser_filters)

### Process

#### Create a Class Definition

- This class must inherit from FilterBase<T>.
- Create methods for:
- Construction
- Destruction
- configure
- update An example prototype can be seen in include/filters/increment.h

#### Implement the Class

- All the methods must be implemented.
- Add the [pluginlib](https://index.ros.org/p/pluginlib) registration
  macro.
- The source can only be the macro if the class is templated. (See
  filters/include/filters/increment.h and filters/src/increment.cpp in
  the package for a templated example)

#### Add a plugin description file

- Which references the library and class of filter.

#### Add the library to CMakeLists.txt

- This should only need one extra line.

#### Export the description file

- Add this line into the export section of manifest.xml for your package

### Files

#### include/filters/increment.h



```cplusplus
#ifndef FILTERS_INCREMENT_H
#define FILTERS_INCREMENT_H
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include "filters/filter_base.h"

namespace filters {

class IncrementFilter: public FilterBase <int>
{
public:
  IncrementFilter();
  ~IncrementFilter();
  virtual bool configure();
  virtual bool update( const T & data_in, T& data_out);
};

}

#endif
```



#### src/increment.cpp



```cplusplus
#include "filters/increment.h"
#include "pluginlib/class_list_macros.h"
using namespace filters;
IncrementFilter::IncrementFilter()
{}

bool IncrementFilter::configure()
{  return true;}

template <typename T>
IncrementFilter::~IncrementFilter()
{}

bool IncrementFilter::update(const int & data_in, int& data_out)
{
  data_out = data_in + 1;
  return true;
};

PLUGINLIB_REGISTER_CLASS(IncrementFilterInt, filters::IncrementFilter<int>, filters::FilterBase<int>)
```



#### plugin_description_file.xml



```text
<class_libraries>
  <library path="lib/libincrement">
    <class name="IncrementFilterInt" type="filters::IncrementFilter<int>"
            base_class_type="filters::FilterBase<int>">
      <description>
        This is a increment filter which works on a stream of ints.
      </description>
    </class>
  </library>
</class_libraries>
```



#### CMakeLists.txt



```cmake
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_add_boost_directories()

#Plugins                                                                                                                                                     
rosbuild_add_library(increment src/increment.cpp)
```



#### manifest.xml



```xml
<package>
<description brief='A standardized filter interface'>

This is an example filter providing package

</description>
<author>Tully Foote/tfoote@willowgarage.com</author>
<license>BSD</license>
<review status="unreviewed" notes=""/>
<url>http://pr.willowgarage.com</url>
<depend package="rosconsole" />
<depend package="roscpp" />
<depend package="pluginlib" />
<export>
  <filters plugin="${prefix}/plugin_description_file.xml" />
</export>
</package>
```
