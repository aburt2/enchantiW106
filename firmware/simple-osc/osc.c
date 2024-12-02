#include "osc.h"

void error(int num, const char *msg, const char *path) {
    printf("Liblo server error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}

int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, lo_message data, void *user_data) {
    for (int i = 0; i < argc; i++) {
        printf("arg %d '%c' ", i, types[i]);
        lo_arg_pp((lo_type)types[i], argv[i]);
        printf("\n");
    }
    printf("\n");
    fflush(stdout);

    return 1;
}

void osc_bundle_add_int(lo_bundle puara_bundle,const char *path, int value) {
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_int32(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(puara_bundle, path, tmp_osc);
}
void osc_bundle_add_float(lo_bundle puara_bundle,const char *path, float value) {
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_float(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(puara_bundle, path, tmp_osc);
}
void osc_bundle_add_int_array(lo_bundle puara_bundle,const char *path, int size, int *value) {
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_int32(tmp_osc, value[i]);
    }
    lo_bundle_add_message(puara_bundle, path, tmp_osc);
}

void osc_bundle_add_float_array(lo_bundle puara_bundle,const char *path, int size,  float *value) {
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_float(tmp_osc, value[i]);
    }
    lo_bundle_add_message(puara_bundle, path, tmp_osc);
}
