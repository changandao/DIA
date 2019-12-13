//
// Created by font_al on 1/14/19.
//

#ifndef BACKEND_H
#define BACKEND_H

#include "frame.h"

namespace fv{

    class BackEnd {

    public:

        // Keyframes
        std::vector<Frame*>* keyframes{};
        size_t* numKeyframes{};

        BackEnd(std::vector<Frame*>* keyframes_, size_t* numKeyframes_):keyframes{keyframes_}, numKeyframes{numKeyframes_}{};

    };
}

#endif //BACKEND_H
