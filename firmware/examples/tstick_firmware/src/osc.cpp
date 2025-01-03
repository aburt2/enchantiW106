#include "osc.hpp"

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

void oscBundle::add(int *idx, const char *path, lo_message msg) {
    int ret = lo_bundle_add_message(bundle, path, msg);
    if (ret < 0) {
        return;
    }
    *idx = num_messages;
    msg_size[num_messages] = lo_strsize(path) + lo_strsize(msg->types);
    msg_length[num_messages] = lo_message_length(msg, path);
    num_messages++;
}

void oscBundle::add(int *idx, const char *path, int value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_int32(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    // Add message to bundle
    add(idx, oscNamespace.c_str(), tmp_osc);
}
void oscBundle::add(int *idx, const char *path, float value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_float(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    // Add message to bundle
    add(idx, oscNamespace.c_str(), tmp_osc);
}
void oscBundle::add(int *idx, const char *path, size_t size, int *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (size_t i = 0; i < size; i++) {
        lo_message_add_int32(tmp_osc, value[i]);
    }
    // Add message to bundle
    add(idx, oscNamespace.c_str(), tmp_osc);
}

void oscBundle::add(int *idx, const char *path, size_t size,  float *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (size_t i = 0; i < size; i++) {
        lo_message_add_float(tmp_osc, value[i]);
    }
    // Add message to bundle
    add(idx, oscNamespace.c_str(), tmp_osc);
}

void oscBundle::send(lo_address a, lo_server from) {
    // Send data
    if (serialise()) {
        // Send the bundle
        lo_send_serialised_bundle_from(a, from, char_bundle, data_len);
    }
}

void oscBundle::fast_send(lo_address a, lo_server from) {
    // Send data
    if (fast_serialise()) {
        // Send the bundle
        lo_send_serialised_bundle_from(a, from, char_bundle, data_len);
    }
}


int oscBundle::serialise() {
    if (data_len > MAX_BUNDLE_SIZE) {
        return 0;
    }

    if(!lo_bundle_serialise(bundle, &char_bundle, &data_len)) {
        // return 0 uif null pointer was returned (indicates error)
        return 0;
    }
    return 1; // if successful
}

int oscBundle::fast_serialise() {
    if (data_len > MAX_BUNDLE_SIZE) {
        return 0;
    }
    if (data_len == 0) first_time = true;
    if (!lo_bundle_serialise_fast(bundle, &char_bundle, &data_len)) {
        // returns 0 if null pointer was returned
        return 0;
    }
    if (first_time) first_time = false; // skips certain memset functions not needed for multiple serialising runs
    return 1;
}

int oscBundle::serialise_message(int idx, void *pos) {
    size_t msg_len;
    lo_message_serialise(bundle->elmnts[idx].content.message.msg, bundle->elmnts[idx].content.message.path, pos, &msg_len);
    return 1;
}

int oscBundle::lo_bundle_serialise_fast(lo_bundle b, void *to, size_t * size)
{
    size_t s, skip;
    int32_t *bes;
    size_t i;
    char *pos;
    lo_pcast32 be;

    if (!b) {
        if (size) *size = 0;
        return -LO_EINVALIDBUND;
    }

    if (data_len == 0) {
        s = lo_bundle_length(b);
        if (size) {
            *size = s;
        }
    } else {
        s = data_len;
    }

    if (!to) {
        return -LO_EINVALIDARG;
    }
    
    pos = (char*) to;
    if (first_time) {
        strcpy(pos, "#bundle");
        pos += 8;
        be.nl = lo_htoo32(b->ts.sec);
        memcpy(pos, &be, 4);
        pos += 4;
        be.nl = lo_htoo32(b->ts.frac);
        memcpy(pos, &be, 4);
        pos += 4;
    } else {
        pos += 16;
    }
    
    for (i = 0; i < b->len; i++) {
        switch (b->elmnts[i].type) {
            case LO_ELEMENT_MESSAGE:
                lo_message_serialise_fast(i, b->elmnts[i].content.message.msg, b->elmnts[i].content.message.path, pos + 4, &skip);
                break;
            case LO_ELEMENT_BUNDLE:
                lo_bundle_serialise_fast(b->elmnts[i].content.bundle, pos+4, &skip);
                break;
	    }

        bes = (int32_t *) (void *)pos;
        *bes = __builtin_bswap32(skip);
        pos += skip + 4;

        if (pos > (char*) to + s) {
            return -LO_EINVALIDBUND;
        }
    }
    if (pos != (char*) to + s) {
        return -LO_EINVALIDBUND;
    }
    return 1;
}

int oscBundle::lo_message_serialise_fast(int msg_idx, lo_message m, const char *path, void *to, size_t * size)
{
    int argc;
    char *ptr;
    size_t s = msg_length[msg_idx];

    if (size) {
        *size = s;
    }

    if (!to) {
        to = calloc(1, s);
    }

    if (first_time) {
        // Only do this once, we don't have to reformat this for future calls
        memset((char*) to + lo_strsize(path) - 4, 0, 4);   // ensure zero-padding
        strcpy((char*) to, path);
        memset((char*) to + lo_strsize(path) + lo_strsize(m->types) - 4, 0,
            4);
        strcpy((char*) to + lo_strsize(path), m->types);
    }
    ptr = (char*) to + msg_size[msg_idx];
    memcpy(ptr, m->data, m->datalen);

    argc = (int) m->typelen - 1;
    fast_reorder(argc, ptr);
    return 1;
}

inline void fast_reorder(int num, void *data) {
    for (int i = 0; i < num; ++i) {
        *(int32_t *) data = __builtin_bswap32(*(int32_t *) data);
        data += 4;
    }
}