// ------------------------------
// Written by Suleyman TARSUSLU
// ------------------------------
#ifndef DESCRIPTOR_HPP
#define DESCRIPTOR_HPP

#include "types.hpp"

namespace ceng391 {

struct Descriptor {
        int key_id;
        uchar desc[32];
};

struct Match {
        int key_id0;
        int key_id1;
        int distance;
};

}

#endif
