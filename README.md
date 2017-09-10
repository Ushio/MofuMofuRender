# MofuMofuRender for rtcamp5

![image](008_(58 spp).png)

## environment
- windows 10 pro
- visual studio 2015 community Update 3

## openframeworks
git location:
    OF_ROOT/apps/MofuMofuRender
- [openframeworks 0.9.8](http://openframeworks.cc/ja/)

## git dependencies
git submodule update --init

- [fixed_size_function](https://github.com/pmed/fixed_size_function)
- [Optional](https://github.com/akrzemi1/Optional)
- [tinyobjloader](https://github.com/syoyo/tinyobjloader)
- [variant](https://github.com/mapbox/variant)

## addons dependencies
ofxImGui to addons folder
- [ofxImGui](https://github.com/jvcleave/ofxImGui/releases/tag/1.50)

## vcpkg dependencies
> vcpkg install glm:x64-windows tbb:x64-windows plog:x64-windows catch:x64-windows

- [glm](https://glm.g-truc.net/0.9.8/index.html)
- [tbb](https://www.threadingbuildingblocks.org/)
- [plog](https://github.com/SergiusTheBest/plog)
- [catch](https://github.com/philsquared/Catch)


## main
PT_HairNEE/PT_HairNEE.sln
PathTracing, NextEventEstimation Multiple Importance Sampling