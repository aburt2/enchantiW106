#include "osc.hpp"

// get send_data function from liblo
extern "C" int send_data(lo_address a, lo_server from, char *data,
                     const size_t data_len);

int oscBundle::init(const char *baseName) {
    baseNamespace.append(baseName);
    baseNamespace.append("/");
    oscNamespace = baseNamespace;

    // Create bundle
    bundle = lo_bundle_new(LO_TT_IMMEDIATE);

    if (!bundle) {
        return 1; // return 1 on failure
    }

    // Return 0 on success
    return 0;
}

void oscBundle::update(size_t num_msg, message_data *msg_arr) {
    // Update messages
    for (size_t i = 0; i < num_msg; i++) {
        if (msg_arr[i].num_elements == 0) {
            if (msg_arr[i].msg_type == LO_INT32) {
                update_message(i, *(int *)msg_arr[i].data);
            } else if (msg_arr[i].msg_type == LO_FLOAT) {
                update_message(i, *(float *)msg_arr[i].data);
            }
        } else {
            if (msg_arr[i].msg_type == LO_INT32) {
                update_message(i, msg_arr[i].num_elements,(int *)msg_arr[i].data);
            } else if (msg_arr[i].msg_type == LO_FLOAT) {
                update_message(i, msg_arr[i].num_elements,(float *)msg_arr[i].data);
            }
        }
    }
}

void oscBundle::update_message(int i, message_data msg_data) {
    // Update messages
    if (msg_data.num_elements == 0) {
        if (msg_data.msg_type == LO_INT32) {
            update_message(i, *(int *)msg_data.data);
        } else if (msg_data.msg_type == LO_FLOAT) {
            update_message(i, *(float *)msg_data.data);
        }
    } else {
        if (msg_data.msg_type == LO_INT32) {
            update_message(i, msg_data.num_elements,(int *)msg_data.data);
        } else if (msg_data.msg_type == LO_FLOAT) {
            update_message(i, msg_data.num_elements,(float *)msg_data.data);
        }
    }
}

int oscBundle::update_message(int i, size_t size, float *data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, data, sizeof(float)*size);
    return 0;
}

int oscBundle::update_message(int i, size_t size, int *data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, data, sizeof(int)*size);
    return 0;
}

int oscBundle::update_message(int i, int data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, &data, sizeof(int));
    return 0;
}

int oscBundle::update_message(int i, float data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, &data, sizeof(float));
    return 0;
}

void oscBundle::add(const char *path, lo_message msg) {
    int ret = lo_bundle_add_message(bundle, path, msg);
    if (ret < 0) {
        return;
    }
    num_messages++;
}

void oscBundle::add_msgs(size_t num_msg, message_data *msg_arr) {
    // Update messages
    for (size_t i = 0; i < num_msg; i++) {
        if (msg_arr[i].num_elements == 0) {
            if (msg_arr[i].msg_type == LO_INT32) {
                add(msg_arr[i].path, *(int *)msg_arr[i].data);
            } else if (msg_arr[i].msg_type == LO_FLOAT) {
                add(msg_arr[i].path, *(float *)msg_arr[i].data);
            }
        } else {
            if (msg_arr[i].msg_type == LO_INT32) {
                add_array(msg_arr[i].path, msg_arr[i].num_elements,(int *)msg_arr[i].data);
            } else if (msg_arr[i].msg_type == LO_FLOAT) {
                add_array(msg_arr[i].path, msg_arr[i].num_elements,(float *)msg_arr[i].data);
            }
        }
    }
}

void oscBundle::add(const char *path, int value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_int32(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    // Add message to bundle
    add(oscNamespace.c_str(), tmp_osc);
}
void oscBundle::add(const char *path, float value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_float(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    // Add message to bundle
    add(oscNamespace.c_str(), tmp_osc);
}
void oscBundle::add_array(const char *path, size_t size, int *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (size_t i = 0; i < size; i++) {
        lo_message_add_int32(tmp_osc, value[i]);
    }
    // Add message to bundle
    add(oscNamespace.c_str(), tmp_osc);
}

void oscBundle::add_array(const char *path, size_t size,  float *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (size_t i = 0; i < size; i++) {
        lo_message_add_float(tmp_osc, value[i]);
    }
    // Add message to bundle
    add(oscNamespace.c_str(), tmp_osc);
}

void oscBundle::send(lo_address a, lo_server from) {
    // Send data
    char_bundle = (char*) lo_bundle_serialise(bundle, char_bundle, &data_len);

    // Send the bundle
    lo_send_serialised_bundle_from(a, from, char_bundle, data_len);
}

void oscBundle::serialise() {
    lo_bundle_serialise(bundle, char_bundle, &data_len);
}

