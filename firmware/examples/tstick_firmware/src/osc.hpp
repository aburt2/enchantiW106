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
class message_data {
    public:
        lo_type msg_type;
        size_t num_elements;
        void * data;   
        char* path;

        // Constructors
        message_data(const char *msg_path, float *arr, size_t num) : msg_type(LO_FLOAT), num_elements(num), data(NULL), path(NULL) {
            // Copy data to buffer
            data = malloc(sizeof(float)*num);
            memcpy((char *)data, arr, sizeof(sizeof(float)*num));

            // Copy message path
            strcpy(path, msg_path);
        }
        message_data(const char *msg_path, int *arr, size_t num) : msg_type(LO_INT32), num_elements(num), data(NULL), path(NULL) {
            // Copy data to buffer
            data = malloc(sizeof(int)*num);
            memcpy((char *)data, arr, sizeof(sizeof(int)*num));

            // Copy message path
            strcpy(path, msg_path);
        }
        message_data(const char *msg_path, float arr) : msg_type(LO_FLOAT), num_elements(0), data(NULL), path(NULL) {
            // Copy data to buffer
            data = malloc(sizeof(float));
            memcpy((char *)data, &arr, sizeof(sizeof(float)));

            // Copy message path
            strcpy(path, msg_path);
        }
        message_data(const char *msg_path, int arr) : msg_type(LO_INT32), num_elements(0), data(NULL), path(NULL) {
            // Copy data to buffer
            data = malloc(sizeof(int));
            memcpy((char *)data, &arr, sizeof(sizeof(int)));

            // Copy message path
            strcpy(path, msg_path);
        }

        // updata data operators
        void update_data(float *val, size_t data_len) {
            memcpy((char *)data, val, sizeof(sizeof(float)*data_len));
        }
        void update_data(int *val, size_t data_len) {
            memcpy((char *)data, val, sizeof(sizeof(int)*data_len));
        }
        void update_data(float val) {
            memcpy((char *)data, &val, sizeof(sizeof(float)));
        }
        void update_data(int val) {
            memcpy((char *)data, &val, sizeof(sizeof(int)));
        }
};

// Helpers
inline void fast_reorder(int num, void *data);

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
        void update_message(int i, message_data msg_data);
        void update(size_t num_msg, message_data *msg_arr);

        // Add data
        void add(const char *path, int value);
        void add(const char *path, float value);
        void add(const char *path, message_data msg_data);
        void add(const char *path, lo_message msg);
        void add_array(const char *path, size_t size, int *value);
        void add_array(const char *path, size_t size,  float *value);
        void add_msgs(size_t num_msg, message_data *msg_arr);

        // Serialise Data
        int serialise();
        int fast_serialise();
        int serialise_message(int idx, void *pos);
        int lo_bundle_serialise_fast(lo_bundle b, void *to, size_t * size);
        int lo_message_serialise_fast(int msg_idx, lo_message m, const char *path, void *to, size_t * size);

        // Send data
        void send(lo_address a, lo_server from);
};

#endif