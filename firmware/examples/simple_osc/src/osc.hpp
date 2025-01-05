// C++ class for updating liblo messages and bundles
#ifndef _LIBLO_OSC_BUNDLE_H_
#define _LIBLO_OSC_BUNDLE_H_
#include <string>
#include <lo/config.h>
#include <lo/lo_types_internal.h>
#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#define MAX_BUNDLE_SIZE 2048
#define MAX_NUM_MESSAGES 52

// Helpers
inline void fast_reorder(size_t len, int num, void *data);

// Bundle
class oscBundle {
    public:
        // Liblo bundles
        lo_bundle bundle;
        char char_bundle[MAX_BUNDLE_SIZE];
        std::string str_bundle = "#bundle";
        int idx;
        size_t data_len = 0;
        bool first_time = true;

        // Store some information about messages
        size_t msg_size[MAX_NUM_MESSAGES];
        size_t msg_length[MAX_NUM_MESSAGES];
        size_t msg_typelen[MAX_NUM_MESSAGES];

        // Properties
        int num_messages = 0;
        std::string baseNamespace = "/";
        std::string oscNamespace;
    
        // Methods
        int init(const char *baseName);
        int update_message(int i, size_t size, float *data);
        int update_message(int i, size_t size, int *data);
        int update_message(int i, int data);
        int update_message(int i, float data);
        int update_message(int i, lo_timetag data);

        // Add data
        void add(int *idx,const char *path, lo_timetag value);
        void add(int *idx,const char *path, int value);
        void add(int *idx,const char *path, float value);
        void add(int *idx,const char *path, size_t size, int *value);
        void add(int *idx,const char *path, size_t size,  float *value);
        void add(int *idx,const char *path, lo_message msg);

        // Serialise Data
        int serialise();
        int fast_serialise();
        int serialise_message(int idx, void *pos);
        int lo_bundle_serialise_fast(lo_bundle b, void *to, size_t * size);
        int lo_message_serialise_fast(int msg_idx, lo_message m, const char *path, void *to, size_t * size);

        // Send data
        void send(lo_address a, lo_server from);
        void fast_send(lo_address a, lo_server from);
};

#endif