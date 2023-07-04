//
// Created by lhx on 16-5-27.
//

#ifndef SROS_VERSION_H
#define SROS_VERSION_H

#define VERSION(major, minor, patch) ((major * 1000 * 1000) + minor * 1000 + patch)

constexpr uint32_t SROS_MAJOR_VERSION = 4;
constexpr uint32_t SROS_MINOR_VERSION = 13;
constexpr uint32_t SROS_PATCH_VERSION = 0;

constexpr uint32_t SROS_VERSION = VERSION(4, 13, 0);

constexpr char SROS_VERSION_STR [] = {
    "4.13.0(67e4495)[feature/laser-correction-test] " __DATE__ " " __TIME__
};

constexpr char GIT_VERSION_STR [] = {
    "67e449537ed8f3191c6381e5291673895f370f29"
};

constexpr char COMPILER_INFO_STR [] = {
    "gcc9"
};

#endif //SROS_VERSION_H
