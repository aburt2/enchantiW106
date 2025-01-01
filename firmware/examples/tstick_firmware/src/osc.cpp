#include "osc.hpp"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
// Logger
LOG_MODULE_REGISTER(OSC);

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
    if (serialise()) {
        // Send the bundle
        lo_send_serialised_bundle_from(a, from, char_bundle, data_len);
    }
}

int oscBundle::serialise() {
    if (data_len > MAX_BUNDLE_SIZE) {
        return 0;
    }
    lo_bundle_serialise_fast(bundle, &char_bundle, &data_len);
    return 1;
}

int oscBundle::serialise_message(int idx, void *pos) {
    size_t msg_len;
    lo_message_serialise(bundle->elmnts[idx].content.message.msg, bundle->elmnts[idx].content.message.path, pos, &msg_len);
    return 1;
}



void *lo_bundle_serialise_fast(lo_bundle b, void *to, size_t * size)
{
    size_t s, skip;
    int32_t *bes;
    size_t i;
    char *pos;
    lo_pcast32 be;

    if (!b) {
        if (size)
            *size = 0;
        return NULL;
    }

    s = lo_bundle_length(b);
    if (size) {
        *size = s;
    }

    if (!to) {
        to = calloc(1, s);
        LOG_INF("Allocated %d bytes to bundle: ", s);
    }

    pos = (char*) to;
    strcpy(pos, "#bundle");
    pos += 8;

    be.nl = lo_htoo32(b->ts.sec);
    memcpy(pos, &be, 4);
    pos += 4;
    be.nl = lo_htoo32(b->ts.frac);
    memcpy(pos, &be, 4);
    pos += 4;
    
    for (i = 0; i < b->len; i++) {
	switch (b->elmnts[i].type) {
	    case LO_ELEMENT_MESSAGE:
		lo_message_serialise(b->elmnts[i].content.message.msg, b->elmnts[i].content.message.path, pos + 4, &skip);
		break;
	    case LO_ELEMENT_BUNDLE:
		lo_bundle_serialise(b->elmnts[i].content.bundle, pos+4, &skip);
		break;
	}

	bes = (int32_t *) (void *)pos;
	*bes = lo_htoo32(skip);
	pos += skip + 4;

	if (pos > (char*) to + s) {
            fprintf(stderr, "liblo: data integrity error at message %lu\n",
                    (long unsigned int)i);

	    return NULL;
	}
    }
    if (pos != (char*) to + s) {
        fprintf(stderr, "liblo: data integrity error\n");
        if (to) {
            free(to);
        }
        return NULL;
    }

    return to;
}