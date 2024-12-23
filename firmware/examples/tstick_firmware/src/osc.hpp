// C++ class for updating liblo messages and bundles
#ifndef _LIBLO_OSC_BUNDLE_H_
#define _LIBLO_OSC_BUNDLE_H_
#include <string>
#include <lo/lo_types_internal.h>
#include <lo/lo_lowlevel.h>

// Bundle
class oscBundle {
    public:
        // Liblo bundles
        lo_bundle bundle;

        // Properties
        int num_messages;
        std::string baseNamespace = "/";
        std::string oscNamespace;
    
        // Methods
        int init(const char *baseName);
        int update_message(int i, void *data);

        // Add data
        void add(const char *path, int value);
        void add(const char *path, float value);
        void add_array(const char *path, int size, int *value);
        void add_array(const char *path, int size,  float *value);
};

#endif